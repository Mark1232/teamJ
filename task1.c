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

    int counter;

    /* Infinite loop. */
    while (1) {
    	for (int i = 0; i<8;i++){
    		prox_values[i] = get_calibrated_prox(i); // Check if calibration works
    	}
        
        // If obsticle detected ahead
    	if (prox_values[0] > 150 || prox_values[7] > 150) {

    		if (prox_values[0] < prox_values[7]){
    	    	left_motor_set_speed(500);
    	    	right_motor_set_speed(-600);
                counter++;
    		}

    		if (prox_values[0] > prox_values[7]){
    		    left_motor_set_speed(-500);
    		    right_motor_set_speed(600);
                counter++;
    		}

            if(counter == 3){
                // Do a 180
                //left_motor_set_speed(-500);
    		    //right_motor_set_speed(600);
                chThdSleepMilliseconds(500);
                counter = 0;
            }

    		chThdSleepMilliseconds(500);
    	}
    	else {
    		left_motor_set_speed(600);
    		right_motor_set_speed(600);
            counter = 0;
    	}
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}