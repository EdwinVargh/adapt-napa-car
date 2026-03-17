#include "arbitrator.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>
#include "motor_driver.h"
#include "esp_adc/adc_oneshot.h"

extern volatile int16_t supervisor_x;
extern volatile int16_t supervisor_y;
extern volatile bool supervisor_hard_override;
extern volatile bool supervisor_emergency_stop;
extern volatile bool controller_connected;

extern adc_oneshot_unit_handle_t adc1_handle; // Declare the ADC handle as extern to use it in this file

// Slew Rate Limiter For Smooth Acceleration/Deceleration
static int16_t apply_slew_limit(int16_t target_val, int16_t current_val, int16_t max_change) {
    // 1. Compute how far we need to move
    int32_t delta = target_val - current_val;
    
    // 2. Clamp the change to our maximum allowed acceleration/deceleration
    if (delta > max_change) {
        delta = max_change;
    } else if (delta < -max_change) {
        delta = -max_change;
    }
    
    // 3. Return the new smoothed output
    return current_val + (int16_t)delta;
}

// Freertos task that continuously reads the supervisor state and controls the car accordingly
void car_control_task(void *pvParameters) {
    int16_t current_motor_x = 2048; // Start at neutral position
    int16_t current_motor_y = 2048; // Start at neutral position

    // At 100Hz loop speed, 20 units per loop = 1.0 second from 100% to 0%.
    int max_step = 20;
    
    while (1)
    {

        if (controller_connected == false) {
            // If the controller is not connected, ensure the motors are stopped and skip the rest of the loop
            kill_motors();
            vTaskDelay(pdMS_TO_TICKS(100)); // Delay to avoid busy looping while waiting for controller connection
            continue;
        }

        int child_x = 0;
        int child_y = 0;
        
        // Read the ADC values for steeringX and steeringY and store them in child_x and child_y
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &child_x));
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_3, &child_y));
        
        // Determine if the supervisor is actively moving the joystick (soft override) even if the trigger is not pulled
        bool supervisor_soft_override = (supervisor_x > 2248 || supervisor_x < 1848 || 
                                         supervisor_y > 2248 || supervisor_y < 1848);

        int target_motor_x = 2048; // Default to stop
        int target_motor_y = 2048; 

        if (supervisor_emergency_stop) {
            // PRIORITY 0: Emergency Stop. If engaged, it overrides EVERYTHING and forces the car to stop immediately.
            // Physically kill power to motor drivers with the enable pins
            kill_motors();
            // Zero out the PWM for extra safety
            current_motor_x = 2048;
            current_motor_y = 2048;
            target_motor_x = 2048;
            target_motor_y = 2048;
        } else {
            // SYSTEM NORMAL: Re-enable the motor drivers
            enable_motors();

            if (supervisor_hard_override) {
                // PRIORITY 1: Trigger is pulled. Supervisor takes FULL control. 
                // Even if the supervisor joystick is centered, it will force the car to stop.
                target_motor_x = supervisor_x;
                target_motor_y = supervisor_y;

            } else if (supervisor_soft_override) {
                // PRIORITY 2: Trigger is NOT pulled, but supervisor is moving the joystick.
                // Override the kid with the supervisor's movement.
                target_motor_x = supervisor_x;
                target_motor_y = supervisor_y;

            } else {
                // PRIORITY 3: Supervisor is doing nothing. Kid has full control.
                if (child_x < 15 || child_x > 4081 || child_y < 15 || child_y > 4081) {
                    // If the ADC reads an impossible extreme, a wire is unplugged or broken.
                    // Force all motors to stop immediately
                    current_motor_x = 2048;
                    current_motor_y = 2048;
                    target_motor_x = 2048;
                    target_motor_y = 2048;
                    
                    supervisor_emergency_stop = true; // Engage emergency stop to prevent potential runaway car due to hardware fault

                    //kill_motors();
                    ///printf("HARDWARE FAULT DETECTED: ADC reading out of bounds! child_x: %d, child_y: %d\n", child_x, child_y);
                    //vTaskDelay(pdMS_TO_TICKS(100)); // Delay to avoid busy looping in case of hardware fault
                    //continue; // Skip the rest of the loop to avoid using invalid values
                }
                target_motor_x = child_x;
                target_motor_y = child_y;
            }
            

        }
        
        if (!supervisor_emergency_stop) {
            // Apply slew rate limiting to the motor outputs for smooth acceleration/deceleration
            current_motor_x = apply_slew_limit(target_motor_x, current_motor_x, max_step);
            current_motor_y = apply_slew_limit(target_motor_y, current_motor_y, max_step);
        }

        // Send the final motor commands to the motor driver
        drive_motors(current_motor_x, current_motor_y);

        
        // remove later
        //debug_drive_pins(target_motor_x);
        //printf("Output: (%d, %d) | Supervisor: (%d, %d) | Child: (%d, %d) | Emergency Stop: %d\n", 
         //       current_motor_x, current_motor_y, supervisor_x, supervisor_y, child_x, child_y, supervisor_emergency_stop);

        vTaskDelay(pdMS_TO_TICKS(10)); // Delay every 10ms to let other tasks run and to avoid hogging the CPU
    }
    
}