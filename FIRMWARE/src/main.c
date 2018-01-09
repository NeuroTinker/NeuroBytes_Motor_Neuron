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

typedef enum{
CR = 	0,
STD = 	1
} servo_type_t;

typedef enum{
    CURRENT     =   0b0001,
    DEND1       =   0b0010,
    DEND2       =   0b0011,
	DEND3       =   0b0100,
	PWM_SPAN	=	0b0101,
	PWM_ZERO	=	0b0111
} parameter_identifiers;

int main(void)
{
	uint32_t	blink_time = 0;
	uint32_t	wait_time = 0;
	uint16_t	data_time = 0;
	uint16_t	send_ping_time = 0;

	// Button debounce variables
	uint16_t	button_press_time = 0;
	uint8_t		button_armed = 0;
	uint16_t	button_status = 0;

	// NID channel (0 = unselected)
	uint32_t	nid_channel = 0b000;

	// Current servo type (temporary solution)
	servo_type_t servo_type = STD; 
	uint16_t servo_zero = 280; // CR = 280
	uint16_t servo_span = 40; // CR = 40

	// Comms helper variables
	uint16_t	lpuart_setup_time = 0;
	uint8_t		change_nid_time = 0;
	message_t	message; // for constructing messages to send to the communications routine

	neuron_t 	neuron;
	uint8_t		i;
	neuronInit(&neuron);
	// TODO: change the neuron init to be unique to each NB board
	neuron.dendrites[0].magnitude = 3000;
	neuron.dendrites[1].magnitude = 3000;
	neuron.dendrites[2].magnitude = 3000;
	neuron.dendrites[0].base_magnitude = 3000;
	neuron.dendrites[1].base_magnitude = 3000;
	neuron.dendrites[2].base_magnitude = 3000;
	commInit();

	clock_setup();
	systick_setup(100); // comm clock in microseconds
	gpio_setup();
	tim_setup();
	gpio_setup();

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
				Check to see if any comms flags have been set.
				Process any data that came from the NID.
			*/

			if (comms_flag != 0){
				switch (comms_flag){
					case DEND1:
						neuron.dendrites[0].magnitude = comms_data;
						break;
					case DEND2:
						neuron.dendrites[1].magnitude = comms_data;
						break;
					case DEND3:
						neuron.dendrites[2].magnitude = comms_data;
						break;
					case PWM_SPAN:
						servo_span = comms_data;
						break;
					case PWM_ZERO:
						servo_zero = comms_data;
						break;
					default:
						break;
				}
				comms_flag = 0;
				comms_data = 0;
			}

			/*
				nid_channel is the current channel, if any, that the NeuroByte is using to communicate
				with the NID. nid_channel should be cleared when NID tries to set a new NeuroByte to 
				identify_channel.

				The communication routine sets identify_time to zero when a new identify command is received.
			*/

			// check for clear channel command
			if (identify_time < IDENTIFY_TIME){
				if (identify_time == 0){
					if ((identify_channel == 0) || (identify_channel == nid_channel)){
						nid_channel = 0;
					}
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
				} else if (button_armed == 1 && nid_i != NO_NID_I){
					nid_channel = identify_channel;
					identify_time = 1;
					button_armed = 0;
				} else if (button_armed == 2){
					if (servo_type == CR){
						servo_type = STD;
					} else if (servo_type == STD){
						servo_type = CR;
					}
					button_armed = 0;
				}
				button_press_time = 0;
			}
		
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
			if (comms_flag != 0){
				switch (comms_flag){
					case DEND1:
						neuron.dendrites[0].magnitude = comms_data;
						break;
					case DEND2:
						neuron.dendrites[1].magnitude = comms_data;
						break;
					case DEND3:
						neuron.dendrites[2].magnitude = comms_data;
						break;
					case PWM_ZERO:
						servo_zero = comms_data;
						break;
					case PWM_SPAN:
						servo_span = comms_data;
						break;
				}
				comms_flag = 0;
				comms_data = 0;
			}


			
			checkDendrites(&neuron);
			
			dendriteDecayStep(&neuron);
			membraneDecayStep(&neuron);

			neuron.potential = calcNeuronPotential(&neuron);
			neuron.potential += neuron.fire_potential;

			if (servo_type == CR) {
				setServo(0, (int32_t)((neuron.potential / 200) + servo_zero) * 1);
				setServo(1, (int32_t)((neuron.potential / 200) - servo_zero) * -1);
			} else {
				setServo(0, (int32_t)(servo_zero + (neuron.potential / servo_span)));
				setServo(1, (int32_t)((neuron.potential / servo_span) - servo_zero) * -1);
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
			} else if (neuron.state == INTEGRATE){
				if (neuron.potential > 10000){
					setLED(200,0,0);
				} else if (neuron.potential > 0){
					setLED(neuron.potential / 50, 200 - (neuron.potential / 50), 0);
				} else if (neuron.potential < -10000){
					setLED(0,0, 200);
				} else if (neuron.potential < 0){
					setLED(0, 200 + (neuron.potential / 50), -1 * neuron.potential / 50);
				} else{
					setLED(0,200,0);
				}
			}
		}
	}
}
