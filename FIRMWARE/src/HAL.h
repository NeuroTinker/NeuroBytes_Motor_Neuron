#ifndef HAL_H_
#define HAL_H_

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/usart.h>

#include "comm.h"


#define NUM_INPUTS 6
#define HAS_DENDS   true
#define NUM_AXONS   0
#define NUM_DENDS   3
#define COMPLIMENTARY_I(i)  i - (i % 2) + ((i+1) % 2)
#define IS_EXCITATORY(i)        ((i+1) % 2)

#define TIM21   TIM21_BASE

#define LPUART1         LPUART1_BASE

#define PORT_LPUART1_TX GPIOA
#define PORT_LPUART1_RX GPIOA

#define PIN_LPUART1_RX  GPIO13
#define PIN_LPUART1_TX  GPIO14

#define PORT_SERVO1     GPIOA
#define PORT_SERVO2     GPIOA

#define PIN_SERVO1      GPIO9
#define PIN_SERVO2      GPIO10

#define PORT_R_LED      GPIOA
#define PORT_G_LED      GPIOA
#define PORT_B_LED      GPIOA
#define PIN_R_LED		GPIO2 // TIM2_CH3
#define PIN_G_LED		GPIO5 // TIM2_CH1
#define PIN_B_LED		GPIO3 // TIM2_CH4

#define PORT_IDENTIFY	GPIOB
#define PIN_IDENTIFY	GPIO1

#define PORT_DEND1_IN   GPIOA
#define PORT_DEND1_EX   GPIOA
#define PORT_DEND2_IN   GPIOA
#define PORT_DEND2_EX   GPIOA
#define PORT_DEND3_IN   GPIOC
#define PORT_DEND3_EX   GPIOC

#define PIN_DEND1_IN    GPIO7
#define PIN_DEND1_EX    GPIO6
#define PIN_DEND2_IN    GPIO1
#define PIN_DEND2_EX    GPIO0
#define PIN_DEND3_IN    GPIO15
#define PIN_DEND3_EX    GPIO14


#define ACTIVATE_INPUT(I, PIN)   active_input_pins[(I)] = PIN; active_input_tick[(I)] = (read_tick + 2) % 3

#define EEPROM_ADDRESS  0x08080000 // 0x08000080 & 0x08080000
#define SERVO_TYPE_ADDRESS    EEPROM_ADDRESS + 0x00
#define FLASH_PRGKEY1   0x8C9DAEBF
#define FLASH_PRGKEY2   0x13141516
#define FLASH_PRGKEYR   0x10
#define FLASH_PEKEY1    0x89ABCDEF
#define FLASH_PEKEY2    0x02030405
#define FLASH_PEKEYR    0x0C

extern const uint16_t complimentary_pins[NUM_INPUTS];
extern const uint32_t complimentary_ports[NUM_INPUTS];
extern volatile uint16_t active_input_pins[NUM_INPUTS];
extern uint32_t active_input_ports[NUM_INPUTS];
extern volatile uint16_t active_output_pins[NUM_INPUTS];
extern uint32_t active_output_ports[NUM_INPUTS];
extern volatile uint8_t active_input_tick[NUM_INPUTS];

extern volatile uint8_t dendrite_pulse_flag[NUM_INPUTS];
extern volatile uint8_t dendrite_ping_flag[NUM_INPUTS];

extern volatile uint8_t main_tick;
extern volatile uint8_t tick;
static const uint16_t gamma_lookup[1024];

bool checkVersion(uint8_t dev, uint16_t version);
void systick_setup(int xms);
void clock_setup(void);
void gpio_setup(void);
void tim_setup(void);
void lpuart_setup(void);
void LEDFullWhite(void);
void setLED(uint16_t r, uint16_t g, uint16_t b);
void setAsInput(uint32_t port, uint32_t pin);
void setAsOutput(uint32_t port, uint32_t pin);
void setServo(uint8_t servo, int32_t duty);

#endif
