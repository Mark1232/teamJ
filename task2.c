#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>


#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>
#include "leds.h"
#include "spi_comm.h"
#include "selector.h"
#include "motors.h"
#include "sensors/proximity.h"
#include "chprintf.h"
#include "usbcfg.h"
#include "sensors/VL53L0X/VL53L0X.h"



#define RAND_MAX 500

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

int main(void)
{
	int prox_values[8];

    halInit();
    chSysInit();
    mpu_init();
    messagebus_init(&bus, &bus_lock, &bus_condvar);
	usb_start();
	proximity_start();
    motors_init();
    clear_leds();
    spi_comm_start();
    calibrate_ir();
	VL53L0X_start();

    int flag;
	int threshold = 75;

    /* Infinite loop. */
    while (1) {

    	flag = 0;
		// Rotate to find object
		while(VL53L0X_get_dist_mm() > 200){
			// Slowly turn to find object
			left_motor_set_speed(200);
			right_motor_set_speed(-200);
			chThdSleepMilliseconds(100);
		}

		// Approach object slowly
		left_motor_set_speed(200);
		right_motor_set_speed(200);

		// If close to target then stop
		while(!((get_calibrated_prox(7) > threshold) || (get_calibrated_prox(0) > threshold))){
				left_motor_set_speed(0);
				right_motor_set_speed(0);
				set_body_led(1);
				chThdSleepMilliseconds(1000);
		}

//		// Centre on object - not sure if strictly needed
//		while(1){
//			if(get_calibrated_prox(7) > get_calibrated_prox(0)){
//				// Turn left
//				left_motor_set_speed(50);
//				right_motor_set_speed(-50);
//			}
//			else if(get_calibrated_prox(7) < get_calibrated_prox(0)){
//				// Turn Right
//				left_motor_set_speed(-50);
//				right_motor_set_speed(50);
//			}
//			else if((get_calibrated_prox(7) > get_calibrated_prox(0) - 5) || (get_calibrated_prox(0) > get_calibrated_prox(7) + 5)){
//				// Stop if centred
//				left_motor_set_speed(0);
//				right_motor_set_speed(0);
//				void set_body_led(1);
//				chThdSleepMilliseconds(1000);
//				break;
//			}
//		}

		// Track object
		while(1){
			// If close then reverse slowly (maybe add leway before threshold - deadzone) 
			if((get_calibrated_prox(7) > threshold + 10) && (get_calibrated_prox(0) > threshold + 10)){
				left_motor_set_speed(-100);
				right_motor_set_speed(-100);
			}

			// If further away then approach (maybe add leway after threshold) 
			else if((get_calibrated_prox(7) < threshold - 10) && (get_calibrated_prox(0) < threshold - 10)){
				left_motor_set_speed(100);
				right_motor_set_speed(100);
			}
			
			// Might have to set these to more smooth turns i.e. no negatives
			// If object is to the left then turn
			else if(get_calibrated_prox(7) - 10  > get_calibrated_prox(0)){
				// Turn left
				left_motor_set_speed(-50);
				right_motor_set_speed(50);
				chThdSleepMilliseconds(100);
			}

			// If object is to the right then turn
			else if(get_calibrated_prox(0) - 10  > get_calibrated_prox(7)){
				// Turn Right
				left_motor_set_speed(50);
				right_motor_set_speed(-50);
				chThdSleepMilliseconds(100);
			}

			// If object is lost
			else if((get_calibrated_prox(7) < 20) && (get_calibrated_prox(0) < 20)){
				break;
			}

			// Stop
			else{
				left_motor_set_speed(0);
				right_motor_set_speed(0);
				chThdSleepMilliseconds(20);
			}
		}
	}
}


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}