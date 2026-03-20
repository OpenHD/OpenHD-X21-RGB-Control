#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// Deklaration für den Linker
extern "C" void SystemClock_Config(void);

#define LED_PIN     PB6 
#define NUM_LEDS    1

// Deine UART Pins PA9/PA10
HardwareSerial MySerial(PA10, PA9);
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// --- LOGIK ---
#define BOOT_TIMEOUT_MS  30000 
enum Mode { MODE_OFF=0, MODE_STATIC=1, MODE_BLINK=2, MODE_BREATHE=3, MODE_BOOT=4, MODE_DOUBLE_FLASH=5 };

struct LedState {
  uint8_t mode; uint8_t r, g, b; uint8_t speed;
  uint32_t last_tick; bool toggle_flag; int breathe_val; int breathe_dir;
};

LedState state = {MODE_BREATHE, 255, 140, 0, 15, 0, false, 0, 1};
bool boot_watchdog_active = true;
bool is_red_alert = false; 

void updateLedEffect(unsigned long current_time);

void setup() {
  // 1. Takt initialisieren
  SystemClock_Config();

  // 2. Hardware starten
  MySerial.begin(115200);
  strip.begin();
  strip.setBrightness(50);
  strip.show();
}

void loop() {
  unsigned long current_time = millis();

  if (MySerial.available() >= 5) {
    uint8_t buffer[5];
    MySerial.readBytes(buffer, 5);
    state.mode  = buffer[0];
    state.r     = buffer[1];
    state.g     = buffer[2];
    state.b     = buffer[3];
    state.speed = buffer[4];
    state.toggle_flag = false;
    state.breathe_val = 0;
    state.breathe_dir = 1; 
    state.last_tick = current_time;
    boot_watchdog_active = false;
    is_red_alert = false;
    while(MySerial.available()) MySerial.read();
    updateLedEffect(current_time);
  }

  if (boot_watchdog_active && (current_time > BOOT_TIMEOUT_MS)) is_red_alert = true;

  if (is_red_alert) {
    static unsigned long last_red_refresh = 0;
    if (current_time - last_red_refresh > 100) {
       last_red_refresh = current_time;
       strip.setPixelColor(0, strip.Color(255, 0, 0));
       strip.show();
    }
  } else {
    updateLedEffect(current_time);
  }
}

void updateLedEffect(unsigned long current_time) {
  uint8_t r_out=0, g_out=0, b_out=0;
  switch (state.mode) {
    case MODE_STATIC: r_out = state.r; g_out = state.g; b_out = state.b; break;
    case MODE_BLINK:
      if (current_time - state.last_tick > (state.speed * 10)) {
        state.last_tick = current_time; state.toggle_flag = !state.toggle_flag;
      }
      if (state.toggle_flag) { r_out = state.r; g_out = state.g; b_out = state.b; }
      break;
    case MODE_BREATHE:
      if (current_time - state.last_tick > state.speed) {
        state.last_tick = current_time;
        state.breathe_val += state.breathe_dir;
        if (state.breathe_val >= 255) { state.breathe_val = 255; state.breathe_dir = -1; }
        if (state.breathe_val <= 5)   { state.breathe_val = 5;   state.breathe_dir = 1; }
      }
      r_out = (state.r * state.breathe_val) / 255;
      g_out = (state.g * state.breathe_val) / 255;
      b_out = (state.b * state.breathe_val) / 255;
      break;
    case MODE_BOOT:
       if (current_time - state.last_tick > 50) { state.last_tick = current_time; state.toggle_flag = !state.toggle_flag; }
       if (state.toggle_flag) { r_out=255; g_out=255; b_out=255; }
       break;
    case MODE_DOUBLE_FLASH:
       {
         uint32_t cycle_len = state.speed * 10; if (cycle_len < 100) cycle_len = 100;
         uint32_t progress = current_time % cycle_len; uint32_t step = cycle_len / 10; 
         if ((progress < step) || (progress > step * 2 && progress < step * 3)) { r_out = state.r; g_out = state.g; b_out = state.b; }
       }
       break;
  }
  strip.setPixelColor(0, strip.Color(r_out, g_out, b_out));
  strip.show();
}

/**
  * @brief System Clock Configuration für STM32C0 (HSI 48MHz, keine PLL)
  */
extern "C" void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  // 1. Internen HSI auf 48MHz konfigurieren
  // Der C0 nutzt den HSI direkt als Systemtaktquelle
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1; // 48MHz / 1 = 48MHz
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  
  // HIER WAR DER FEHLER: Wir lassen das PLL-Feld komplett weg, 
  // da es im C0 HAL-Struct für diesen Chip nicht existiert.

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    while (1); 
  }

  // 2. CPU, AHB und APB Busse auf den HSI takten
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  // Flash-Latency einstellen (Wichtig bei 48MHz!)
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    while (1);
  }
}