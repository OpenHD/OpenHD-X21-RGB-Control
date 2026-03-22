#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#include "gamma.h"
extern "C" void SystemClock_Config(void);

#define LED_PIN     PB6 
#define NUM_LEDS    1

// SoC Uart at PA9/10
HardwareSerial MySerial(PA10, PA9);
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Boot happens in 3 stages:
// Stage 1 - Powered, no data from SoC
// Stage 2 - Pet from u-boot
// Stage 3 - Pet from userspace

// Userspace is expected to pet each 5s unless
// fatal error animation was sent.

#define BOOT_TIMEOUT_U_BOOT_MS  10000
#define BOOT_TIMEOUT_USERSPACE_MS  50000
#define BOOT_TIMEOUT_USERSPACE_PERIODIC_MS  50000

// Reboot loop detection
// If SoC reboots more then 5 time in a minute (as suggested by uboot pets)
// Consider it an error

#define REBOOT_LOOP_TIMEOUT_MS  600000
#define REBOOT_LOOP_REBOOT_COUNT 5

// UART timeout
#define UART_TIMEOUT 500

// Default soft transition time
#define SOFT_TANSITION_MS 1000

// Color, 24bpp, RGB
struct __attribute__((packed)) color {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

// Animated color, includes color and delay, delay is relevant to
// transition period AFTER the color is set, delay is 16 bit unsigned int in ms
struct __attribute__((packed)) animated_color {
  struct color color;
  uint16_t delay;
};

enum boot_stage {
  PRE_U_BOOT,
  U_BOOT,
  USERSPACE
};

// Commands/modes
// Each command from soc start with unique 1 byte ID
// ends with single byte CRC
// And can be any size within 2-128 bytes including CRC
enum led_mode {
  GET_VERSION,
  LED_OFF,
  LED_ON,
  LED_STATIC_COLOR,
  LED_BREATHE,
  LED_BLINK,
  PET_U_BOOT,
  PET_USERSPACE,
  PET_PERIODIC,
  REBOOT
};

#define MAX_CMD_PAYLOAD 128

// Command structure

// GET_VERSION is used by SoC to request this firmware version
// no arguments

// LED_OFF turns off LED while keeping it's last state (any animation is stopped)
// Any commands to set color/animation will set the state but wont turn LED ON
// no arguments

// LED_ON turns on LED and restores last state/restarts animation
// no arguments

// LED_STATIC_COLOR sets the provided color
// color - Color to be set
// soft_transition - whether initial transition to 1st new state should be gradual
// keep_across_reboot - whether to keep state during next reboot (failure to pet will result in failure anyway)
// fatal_fault - will keep this state PERMANENTLY until MCU reboot

struct __attribute__((packed)) cmd_set_static_color {
  struct color color;
  bool soft_transition;
  bool keep_across_reboot;
  bool fatal_fault;
};

// LED_BREATHE enables breathing animation
// unlike blinking it does soft transition between provide colors
// len - count of states
// soft_transition - whether initial transition to 1st new state should be gradual
// keep_across_reboot - whether to keep state during next reboot (failure to pet will result in failure anyway)
// fatal_fault - will keep this state PERMANENTLY until MCU reboot
// anim - array of states

struct __attribute__((packed)) cmd_led_breathe {
  uint8_t len;
  bool soft_transition;
  bool keep_across_reboot;
  bool fatal_fault;
};

// LED_BLINK enables blinking animation
// it changes provided states with provided delays
// len - count of states
// soft_transition - whether initial transition to 1st new state should be gradual
// keep_across_reboot - whether to keep state during next reboot (failure to pet will result in failure anyway)
// fatal_fault - will keep this state PERMANENTLY until MCU reboot
// anim - array of states

struct __attribute__((packed)) cmd_led_blink {
  uint8_t len;
  bool soft_transition;
  bool keep_across_reboot;
  bool fatal_fault;
};

// PET_U_BOOT is triggered by SoC at u-boot stage
// no arguments

// PET_USERSPACE is triggered by SoC when userspace led control process starts
// no arguments

// PET_PERIODIC is triggered by SoC led control process periodically
// no arguments

// REBOOT is triggered by SoC before rebooting to change petting expectation
// no arguments

struct animated_color boot_anim[2] = {{{0, 255, 0}, 1000}, {{0, 0, 0}, 1000}};
struct animated_color u_boot_fault_generic_anim[2] = {{{255, 0, 0}, 1000}, {{0, 0, 0}, 1000}};
struct animated_color os_fault_generic_anim[2] = {{{255, 0, 0}, 1000}, {{100, 100, 0}, 1000}};
struct animated_color os_hang_anim[2] = {{{255, 0, 0}, 1000}, {{0, 255, 0}, 1000}};
struct color booted_color = {0, 255, 0};

struct current_state {
  led_mode mode;
  animated_color anim[128];
  uint8_t anim_len;
  uint8_t anim_id;
  bool soft_transition;
  color current_color;
  animated_color previous_color;
  uint32_t state_time;
  bool on;
  bool fatal_fault;
};

struct current_state state;
struct current_state backup_state;
uint32_t last_pet;
enum boot_stage boot_stage;
bool wdt_triggered;
uint32_t rx_last_packet_time  = 0;
uint8_t buffer[128];
uint16_t final_len = 0;
uint16_t buffer_pos = 0;

uint8_t crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;

    while (len--) {
        crc ^= *data++;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

bool crc8_check(const uint8_t *data, size_t len) {
    if (len == 0)
      return false;

    uint8_t expected_crc = data[len-1];
    uint8_t computed_crc = crc8(data, len-1);

    return expected_crc == computed_crc;
}

void estimate_transition_color(color start, color end, uint32_t timeframe, uint32_t timePassed, color *out) {
    // Better safe then sorry
    if (timePassed >= timeframe) {
        out->r = end.r;
        out->g = end.g;
        out->b = end.b;
        return;
    }

    int32_t sR = (int32_t)start.r;
    int32_t sG = (int32_t)start.g;
    int32_t sB = (int32_t)start.b;

    int32_t dR = (int32_t)end.r - sR;
    int32_t dG = (int32_t)end.g - sG;
    int32_t dB = (int32_t)end.b - sB;

    int32_t t  = (int32_t)timePassed;
    int32_t tf = (int32_t)timeframe;

    out->r = (uint8_t)(sR + (dR * t) / tf);
    out->g = (uint8_t)(sG + (dG * t) / tf);
    out->b = (uint8_t)(sB + (dB * t) / tf);
}

void setup() {
  SystemClock_Config();
  // Init uart at 115200 baud
  MySerial.begin(115200);
  MySerial.setTimeout(10);
  strip.begin();
  strip.setBrightness(50);
  strip.show();
  memcpy(&state.anim, &boot_anim, sizeof(boot_anim));
  state.anim_id = 0;
  state.anim_len = sizeof(boot_anim) / sizeof(struct animated_color);
  state.current_color = {0, 0, 0};
  state.fatal_fault = false;
  state.mode = LED_BREATHE;
  state.on = true;
  state.previous_color = {0, 0, 0};
  state.soft_transition = true;
  state.previous_color = {{0, 0, 0}, 500};
  state.state_time = millis();
  memcpy(&backup_state, &state, sizeof(state));
}

void loop() {
  uint32_t current_time = millis();

  uint16_t last_avalible = MySerial.available();
  if (!state.fatal_fault && (last_avalible >= 2 || (last_avalible >= 1 && final_len > 0))) {
    rx_last_packet_time = current_time;
    uint8_t cmd;
    uint16_t len;
    
    if(final_len == 0) {
      MySerial.readBytes(&cmd, 1);
      buffer[0] = cmd;
      buffer_pos = 1;
      switch(cmd) {
        case LED_STATIC_COLOR: {
          len = sizeof(cmd_set_static_color);
          MySerial.readBytes(&buffer[1], min((uint16_t)(last_avalible - 1), len));
          buffer_pos += min((uint16_t)(last_avalible - 1), len);
          final_len = len;
          // CMD
          final_len++;
          // CRC
          final_len++;
          break;
        }
        case LED_BREATHE: {
          MySerial.readBytes(&buffer[1], 1);
          if(buffer[1] > MAX_CMD_PAYLOAD) {
            len = 0;
            final_len = 0;
            buffer_pos = 0;
            memset(buffer, 0, sizeof(buffer));
            while(MySerial.available()) MySerial.read();
            goto drive_led;
          }
          len = buffer[1] * sizeof(animated_color);
          len += sizeof(cmd_led_breathe);
          buffer_pos += 1;
          final_len = len;
          // CMD
          final_len++;
          // CRC
          final_len++;
          MySerial.readBytes(&buffer[2], min(last_avalible - 2, final_len - buffer_pos));
          buffer_pos +=  min(last_avalible - 2, final_len - buffer_pos);
          break;
        }
        case LED_BLINK: {
          MySerial.readBytes(&buffer[1], 1);
          if(buffer[1] > MAX_CMD_PAYLOAD) {
            len = 0;
            final_len = 0;
            buffer_pos = 0;
            memset(buffer, 0, sizeof(buffer));
            while(MySerial.available()) MySerial.read();
            goto drive_led;
          }
          len = buffer[1] * sizeof(animated_color);
          len += sizeof(cmd_led_blink);
          buffer_pos += 1;
          final_len = len;
          // CMD
          final_len++;
          // CRC
          final_len++;
          MySerial.readBytes(&buffer[2], min(last_avalible - 2, final_len - buffer_pos));
          buffer_pos +=  min(last_avalible - 2, final_len - buffer_pos);
          break;
        }
        default: {
          final_len = 2;
          MySerial.readBytes(&buffer[1], 1);
          buffer_pos = 2;
          break;
        }
      }
    } else {
      len = final_len - buffer_pos;
      MySerial.readBytes(&buffer[buffer_pos], min(len, last_avalible));
      buffer_pos += min(len, last_avalible);
    }

    if (final_len == buffer_pos) {
      if(!crc8_check(buffer, final_len)) {
        len = 0;
        final_len = 0;
        buffer_pos = 0;
        memset(buffer, 0, sizeof(buffer));
        while(MySerial.available()) MySerial.read();
        goto drive_led;
      }
      switch(buffer[0]) {
        case LED_OFF: {
          state.on = false;
          break;
        }
        case LED_ON: {
          state.on = true;
          break;
        }
        case LED_STATIC_COLOR: {
          // Offset by 1 to account for cmd id
          cmd_set_static_color *tmp = (cmd_set_static_color*)&buffer[1];
          state.mode = LED_STATIC_COLOR;
          state.anim_id = 0;

          memset(&state.anim, 0, sizeof(state.anim));
          state.previous_color.color = state.current_color;
          state.anim[0] = {tmp->color, 0};
          state.anim_len = 1;
          state.fatal_fault = tmp->fatal_fault;
          if(tmp->soft_transition) {
            state.previous_color.color = state.current_color;
            state.previous_color.delay = SOFT_TANSITION_MS;
            state.soft_transition = true;
            state.anim_id = 0;
            state.state_time = current_time;
          } else {
            state.soft_transition = false;
            state.current_color = state.anim[0].color;
            state.anim_id = 0;
            state.state_time = current_time;
          }
          break;
        }
        case LED_BREATHE: {
          // Offset by 1 to account for cmd id
          cmd_led_breathe *tmp = (cmd_led_breathe*)&buffer[1];
          animated_color *rx_anim = (animated_color*)&buffer[1 + sizeof(cmd_led_breathe)];
          state.mode = LED_BREATHE;
          state.anim_id = 0;

          memset(&state.anim, 0, sizeof(state.anim));
          state.previous_color.color = state.current_color;
          memcpy(state.anim, rx_anim, sizeof(animated_color) * tmp->len);
          state.anim_len = tmp->len;
          state.fatal_fault = tmp->fatal_fault;
          if(tmp->soft_transition) {
            state.previous_color.color = state.current_color;
            state.previous_color.delay = SOFT_TANSITION_MS;
            state.soft_transition = true;
            state.anim_id = 0;
            state.state_time = current_time;
          } else {
            state.soft_transition = false;
            state.current_color = state.anim[0].color;
            state.previous_color = state.anim[0];
            state.anim_id = (state.anim_len > 1) ? 1 : 0;
            state.state_time = current_time;
          }
          break;
        }
        case LED_BLINK: {
          // Offset by 1 to account for cmd id
          cmd_led_blink *tmp = (cmd_led_blink*)&buffer[1];
          animated_color *rx_anim = (animated_color*)&buffer[1 + sizeof(cmd_led_blink)];
          state.mode = LED_BLINK;
          state.anim_id = 0;

          memset(&state.anim, 0, sizeof(state.anim));
          state.previous_color.color = state.current_color;
          memcpy(state.anim, rx_anim, sizeof(animated_color) * tmp->len);
          state.anim_len = tmp->len;
          state.fatal_fault = tmp->fatal_fault;
          if(tmp->soft_transition) {
            state.previous_color.color = state.current_color;
            state.previous_color.delay = SOFT_TANSITION_MS;
            state.soft_transition = true;
            state.anim_id = 0;
            state.state_time = current_time;
          } else {
            state.soft_transition = false;
            state.current_color = state.anim[0].color;
            state.previous_color = state.anim[0];
            state.anim_id = (state.anim_len > 1) ? 1 : 0;
            state.state_time = current_time;
          }
          break;
        }
        case PET_U_BOOT: {
          if(boot_stage == PRE_U_BOOT) {
            boot_stage = U_BOOT;
            last_pet = current_time;
          } else if (boot_stage == U_BOOT) {
            last_pet = current_time;
          }
          // In any other case message is malformed
          break;
        }
        case PET_USERSPACE: {
          if(boot_stage == U_BOOT || boot_stage == PRE_U_BOOT) {
            boot_stage = USERSPACE;
            last_pet = current_time;
            state.mode = LED_STATIC_COLOR;
            state.anim_id = 0;

            memset(&state.anim, 0, sizeof(state.anim));
            state.previous_color.color = state.current_color;
            state.anim[0] = {booted_color, 0};
            state.anim_len = 1;
            state.previous_color.delay = SOFT_TANSITION_MS;
            state.soft_transition = true;
            state.state_time = current_time;
          }
          // In any other case message is malformed
          break;
        }
        case PET_PERIODIC: {
          last_pet = current_time;
          break;
        }
      }

      len = 0;
      final_len = 0;
      buffer_pos = 0;
      memset(buffer, 0, sizeof(buffer));
    }

  } else {
    if(current_time - rx_last_packet_time > UART_TIMEOUT) {
      final_len = 0;
      buffer_pos = 0;
      memset(buffer, 0, sizeof(buffer));
    }
  }
drive_led:
  // Handle WDT style tasks
  if(!state.fatal_fault) {
    switch(boot_stage) {
      case PRE_U_BOOT: {
        if(current_time - last_pet >= BOOT_TIMEOUT_U_BOOT_MS && !wdt_triggered) {
          // State is default one, no need to back up
          memset(&state.anim, 0, sizeof(state.anim));
          state.mode = LED_BREATHE;
          state.previous_color.color = state.current_color;
          state.previous_color.delay = SOFT_TANSITION_MS;
          state.soft_transition = true;
          state.state_time = current_time;
          memcpy(&state.anim, u_boot_fault_generic_anim, sizeof(u_boot_fault_generic_anim));
          state.anim_id = 0;
          state.anim_len = sizeof(u_boot_fault_generic_anim) / sizeof(animated_color);
          wdt_triggered = true;
        } else if(current_time - last_pet <= BOOT_TIMEOUT_U_BOOT_MS && wdt_triggered) {
          memset(&state.anim, 0, sizeof(state.anim));
          state.previous_color.color = state.current_color;
          state.previous_color.delay = SOFT_TANSITION_MS;
          state.soft_transition = true;
          state.state_time = current_time;
          memcpy(&state.anim, boot_anim, sizeof(boot_anim));
          state.anim_id = 0;
          state.anim_len = sizeof(boot_anim) / sizeof(animated_color);
          wdt_triggered = false;
        }
        break;
      }
      case U_BOOT: {
        if(current_time - last_pet >= BOOT_TIMEOUT_USERSPACE_MS && !wdt_triggered) {
          // State is default one, no need to back up
          memset(&state.anim, 0, sizeof(state.anim));
          state.mode = LED_BREATHE;
          state.previous_color.color = state.current_color;
          state.previous_color.delay = SOFT_TANSITION_MS;
          state.soft_transition = true;
          state.state_time = current_time;
          memcpy(&state.anim, os_fault_generic_anim, sizeof(os_fault_generic_anim));
          state.anim_id = 0;
          state.anim_len = sizeof(os_fault_generic_anim) / sizeof(animated_color);
          wdt_triggered = true;
        } else if(current_time - last_pet <= BOOT_TIMEOUT_USERSPACE_MS && wdt_triggered) {
          memset(&state.anim, 0, sizeof(state.anim));
          state.previous_color.color = state.current_color;
          state.previous_color.delay = SOFT_TANSITION_MS;
          state.soft_transition = true;
          state.state_time = current_time;
          memcpy(&state.anim, boot_anim, sizeof(boot_anim));
          state.anim_id = 0;
          state.anim_len = sizeof(boot_anim) / sizeof(animated_color);
          wdt_triggered = false;
        }
        break;
      }
      case USERSPACE: {
        if(current_time - last_pet >= BOOT_TIMEOUT_USERSPACE_PERIODIC_MS && !wdt_triggered) {
          // Backup the state
          memcpy(&backup_state, &state, sizeof(state));
          memset(&state.anim, 0, sizeof(state.anim));
          state.mode = LED_BREATHE;
          state.previous_color.color = state.current_color;
          state.previous_color.delay = SOFT_TANSITION_MS;
          state.soft_transition = true;
          state.state_time = current_time;
          memcpy(&state.anim, os_hang_anim, sizeof(os_hang_anim));
          state.anim_id = 0;
          state.anim_len = sizeof(os_hang_anim) / sizeof(animated_color);
          wdt_triggered = true;
        } else if(current_time - last_pet <= BOOT_TIMEOUT_USERSPACE_PERIODIC_MS && wdt_triggered) {
          backup_state.previous_color.color = state.current_color;
          backup_state.previous_color.delay = SOFT_TANSITION_MS;
          backup_state.soft_transition = true;
          backup_state.state_time = current_time;
          memcpy(&state, &backup_state, sizeof(state));
          wdt_triggered = false;
        }
        break;
      }
    }
  }
  // Calculate new state
  // Calculate absolute strate time
  uint32_t state_time = current_time - state.state_time;

  if(state.soft_transition) {
    if(state_time >= state.previous_color.delay) {
      state.soft_transition = false;
      state.current_color = state.anim[0].color;
      state.previous_color.color = state.anim[0].color;
      state.anim_id = (state.anim_len > 1) ? 1 : 0;
      state.state_time = current_time;
      goto display;
    } else {
      estimate_transition_color(state.previous_color.color, state.anim[0].color, state.previous_color.delay, state_time, &state.current_color);
      goto display;
    }
  }

  if(state.mode == LED_STATIC_COLOR && !state.soft_transition) {
    state.current_color = state.anim[0].color;
  }

  if(state.mode == LED_BREATHE && !state.soft_transition) {
    uint8_t next_state = state.anim_id + 1;
    if(next_state >= state.anim_len) {
      next_state = 0;
    }
    if(state_time >= state.anim[state.anim_id].delay) {
      state.previous_color = state.anim[state.anim_id];
      state.current_color = state.anim[state.anim_id].color;
      state.anim_id = next_state;
      state.state_time = current_time;
    } else {
      estimate_transition_color(state.previous_color.color, state.anim[state.anim_id].color, state.previous_color.delay, state_time, &state.current_color);
    }
  }

  if(state.mode == LED_BLINK && !state.soft_transition) {
    uint8_t next_state = state.anim_id + 1;
    if(next_state >= state.anim_len) {
      next_state = 0;
    }
    if(state_time >= state.anim[state.anim_id].delay) {
      state.previous_color = state.anim[state.anim_id];
      state.current_color = state.anim[state.anim_id].color;
      state.anim_id = next_state;
      state.state_time = current_time;
    }
  }
display:
  if (state.on) {
    strip.setPixelColor(0, strip.Color(gamma8[state.current_color.r], gamma8[state.current_color.g], gamma8[state.current_color.b]));
  } else {
    strip.setPixelColor(0, 0);
  }
  strip.show();
}

/**
  * @brief System Clock Configuration for STM32C0 (HSI 48MHz, without PLL)
  */
extern "C" void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    while (1); 
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    while (1);
  }

  SET_BIT(SYSCFG->CFGR1, SYSCFG_CFGR1_PA11_RMP);
  SET_BIT(SYSCFG->CFGR1, SYSCFG_CFGR1_PA12_RMP);
}