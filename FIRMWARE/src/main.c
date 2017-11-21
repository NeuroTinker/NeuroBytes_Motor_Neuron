#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/exti.h>

#include "HAL.h"
#include "comm.h"
#include "neuron.h"

#define BLINK_TIME			100
#define DATA_TIME			10
#define DEND_PING_TIME		200 // 1000 ms
#define	NID_PING_TIME		200 // 1000 ms
#define SEND_PING_TIME		80
#define BUTTON_PRESS_TIME	2
#define BUTTON_HOLD_TIME	100

static uint32_t fingerprint[3] __attribute__((section (".fingerprint"))) __attribute__ ((__used__)) = {
	3, // device id
	1, // firmware version
	0  // unique id
};

int main(void)
{
	uint32_t	blink_time = 0;
	uint32_t	wait_time = 0;
	uint16_t	data_time = 0;
	uint16_t	message_data = 0;
	uint16_t	send_ping_time = 0;
	uint16_t	button_press_time = 0;
	uint8_t		button_armed = 0;
	uint16_t	button_status = 0;
	uint32_t	nid_channel = 0b000;
	uint32_t	message = 0;

	neuron_t 	neuron;
	uint8_t		i;
	neuronInit(&neuron);
	commInit();

	clock_setup();
	systick_setup(100); // comm clock in microseconds
	gpio_setup();
	tim_setup();
	gpio_setup();
	setLED(200,0,0);
	//systick_setup(100000);

	//	MMIO32(SYSCFG_BASE + 0x0c) = 0b1111 << 12;

	
	for(;;)
	{

		if (main_tick == 1){
			// 5 ms
			//setLED(0,200,100);
			main_tick = 0;

			// check identify button
			button_status = gpio_get(PORT_IDENTIFY, PIN_IDENTIFY);
			//button_status |= PIN_IDENTIFY;

			// if identify button is pressed and identify_time < IDENTIFY_TIME (i.e. NID sent 'identify'' message), set new nid_channel
			if (button_status == 0){
				// debounce
				button_press_time += 1;
				if (button_press_time >= BUTTON_HOLD_TIME){
					button_armed = 2;
					blink_flag = 1;
				} else if (button_press_time >= BUTTON_PRESS_TIME){
					button_armed = 1;
				}
			} else{
				// button not pressed
				if (button_armed == 0){
					button_press_time = 0;
				} else if (button_armed == 1){
					if (identify_time < IDENTIFY_TIME){
						nid_channel = identify_channel;
					} else{
						// temporarily use identify button also as an impulse button
						neuron.fire_potential += 11000;
						//neuron.leaky_current += 20;
					}
					button_armed = 0;
				} else if (button_armed == 2){
					if (neuron.servo_type == CR){
						neuron.servo_type = STD;
					} else if (neuron.servo_type == STD){
						neuron.servo_type = CR;
					}
					button_armed = 0;
				}
				button_press_time = 0;
			}

/*
			button_status = gpio_get(PORT_IDENTIFY, PIN_IDENTIFY);
			if (identify_time > 0){
				identify_time -= 1;
				if (identify_channel == 0){
					// clear all channels
					nid_channel = 0;
				}
			}
			if (button_status == 0){ // !=
				if (button_press_time++ >= BUTTON_PRESS_TIME){
					button_armed = 1;
					button_press_time = 0;
				}
			} else if (button_armed == 1){
				if (identify_time > 0){
					nid_channel = identify_channel;
				} else{
					neuron.fire_potential += 60;
				}
				button_armed = 0;
			} else{
				button_press_time = 0;
			}
*/
			
			if (nid_channel != 0){
				if (data_time++ > DATA_TIME){
					data_time = 0;
					message = DATA_MESSAGE | (uint16_t) neuron.potential | (nid_channel << 19) | (nid_keep_alive << 22);
					addWrite(NID_BUFF,message);
				}
			}
			
			checkDendrites(&neuron);
			
			dendriteDecayStep(&neuron);
			membraneDecayStep(&neuron);

			neuron.potential = calcNeuronPotential(&neuron);
			neuron.potential += neuron.fire_potential;

			if (neuron.servo_type == CR) {
				setServo(0, (int32_t)((neuron.potential / 1000) + SERVO_ZERO) * 1);
				setServo(1, (int32_t)((neuron.potential / 1000) - SERVO_ZERO) * -1);
			} else {
				setServo(0, (int32_t)((neuron.potential / 200) + SERVO_ZERO) * 1);
				setServo(1, (int32_t)((neuron.potential / 200) - SERVO_ZERO) * -1);
			}

			if (blink_flag != 0){
				setLED(200,0,300);
				blink_time = 1;
				blink_flag = 0;
			} else if (blink_time > 0){
				if (++blink_time == BLINK_TIME){
					setLED(200,0,0);
					blink_time = 0;
				}
			} else if (neuron.state == FIRE){
				neuron.fire_time -= 1;
				if (neuron.fire_time == 0){
					neuron.state = INTEGRATE;
				}
				LEDFullWhite();
			} else if (neuron.state == INTEGRATE){
				if (neuron.potential > 40000){
					setLED(200,0,0);
				} else if (neuron.potential > 0){
					if (neuron.servo_type == CR) {
						setLED((neuron.potential/2)/100, 200 - ((neuron.potential/2) / 100), 200 - ((neuron.potential/2) / 100));
					} else {
						setLED(200, 130 - ((neuron.potential/2) / 154), 0);
					}
				} else if (neuron.potential < -40000){
					setLED(0,0, 200);
				} else if (neuron.potential < 0){
					if (neuron.servo_type == CR) {
						setLED(0, 200 + ((neuron.potential/2) / 100), 200);
					} else {
						setLED(200 + ((neuron.potential/2) / 100), 130 + ((neuron.potential/2) / 154), -1 * (neuron.potential /2) / 100);
					}
				} else{
					if (neuron.servo_type == CR) {
						setLED(0,200,200);
					} else {
						setLED(200,130,0);
					}
				}
			}
		}
	}
}
