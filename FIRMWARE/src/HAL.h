#ifndef HAL_H_
#define HAL_H_

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/exti.h>

#include "comm.h"

#define TIM21   TIM21_BASE

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

#define SERVO_ZERO      280



extern volatile uint8_t main_tick;
extern volatile uint8_t tick;
extern volatile uint8_t read_tick;
static const uint16_t gamma_lookup[1024];


// uint8_t     read_time = 0;


void systick_setup(int xms);
void clock_setup(void);
void gpio_setup(void);
void tim_setup(void);
void LEDFullWhite(void);
void setLED(uint16_t r, uint16_t g, uint16_t b);
void setAsInput(uint32_t port, uint32_t pin);
void setAsOutput(uint32_t port, uint32_t pin);
void setServo(uint8_t servo, int32_t duty);


//void tim2_isr(void);

//extern volatile unsigned char ms_tick;

#endif
