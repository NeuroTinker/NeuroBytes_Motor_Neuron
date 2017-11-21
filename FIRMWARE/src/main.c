#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/usart.h>

#include "HAL.h"
#include "comm.h"
#include "neuron.h"

#define BLINK_TIME			40
#define DATA_TIME			10
#define DEND_PING_TIME		200 // 1000 ms
#define	NID_PING_TIME		200 // 1000 ms
#define SEND_PING_TIME		80 // 80
#define BUTTON_PRESS_TIME	2
#define BUTTON_HOLD_TIME    100
#define LPUART_SETUP_TIME	100
#define CHANGE_NID_TIME 	200

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

	for(;;)
	{

		if (main_tick == 1){
			// 5 ms
			main_tick = 0;
			
			// check to see if nid ping hasn't been received in last NID_PING_TIME ticks
			if (nid_ping_time > 0){
				nid_ping_time -= 1;
				if (nid_ping_time == 0){
					// nid no longer connected
					nid_distance = 100; // reset nid_keep_alive
					nid_pin = 0; // clear the nid pin
					nid_pin_out = 0;
					nid_i = 13; // make this a macro like NO_NID_I
				}
			}

			if (change_nid_time++ > CHANGE_NID_TIME){
				change_nid_time = 0;
				closer_distance = nid_distance;
				closer_ping_count = 0;
			}

			if (lpuart_setup_time < LPUART_SETUP_TIME){
				lpuart_setup_time += 1;
			} else if (lpuart_setup_time == LPUART_SETUP_TIME){
				lpuart_setup_time += 1;
				lpuart_setup();
			}

			/*
				nid_channel is the current channel, if any, that the NeuroByte is using to communicate
				with the NID. nid_channel should be cleared when NID tries to set a new NeuroByte to 
				identify_channel.

				The communication routine sets identify_time to zero when a new identify command is received.
			*/

			// check for clear channel command
			if (identify_time < IDENTIFY_TIME){
				 if (identify_channel == 0){ 
				 	// setting identify channel 0 clears identify_channel */
				 	nid_channel = 0; 
				 } else if (identify_channel == nid_channel && identify_time == 0){ 
				 	// clear nid_channel if NID is trying to set a new NeuroByte to the current nid_channel */
				 	nid_channel = 0; 
				 } 
				 identify_time += 1;
			}

			// check identify button
			button_status = gpio_get(PORT_IDENTIFY, PIN_IDENTIFY);

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
			
			// send current membrane potential to NID if currently identified by NID
			if (nid_channel != 0){
				// send data every DATA_TIME ticks
				if (data_time++ > DATA_TIME){
					message.message = (((uint32_t) DATA_MESSAGE)) | ((uint16_t) neuron.potential);						
					data_time = 0;
					message.length = 32;
					message.message |= (nid_channel << 21);
					addWrite(NID_BUFF,(const message_t) message);
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
