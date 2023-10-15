/**
 * @file Tachometer_main.c
 * @brief Main source code for the Tachometer program.
 *
 * This file contains the main entry point for the Tachometer program.
 * It uses Timer A0 to generate PWM signals that will be used to drive the DC motors.
 * Then, it uses the edge-triggered interrupts from the bump sensors to detect a collision,
 * which should immediately stop the motors from running.
 *
 * Other timers are used in this lab:
 *  - Timer A1: Used to generate periodic interrupts at a specified rate (2 kHz)
 *  - Timer A2: Used for input capture to request interrupts on the rising edge of P8.0
 *  - Timer A3: Used for input capture to request interrupts on the rising edge of P10.4 and P10.5
 *              and to calculate the period between captures
 *
 * @author Aaron Nanas
 *
 */

#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/GPIO.h"
#include "../inc/Bumper_Sensors.h"
#include "../inc/EUSCI_A0_UART.h"
#include "../inc/Timer_A0_PWM.h"
#include "../inc/Timer_A1_Interrupt.h"
#include "../inc/Timer_A2_Capture.h"
#include "../inc/Timer_A3_Capture.h"
#include "../inc/Motor.h"
#include "../inc/Tachometer.h"

//#define MAIN_CONTROLLER 1
//#define TASK_1          1
#define TASK_2          1

// Initialize length of the tachometer buffers
#define BUFFER_LENGTH                 10

// Set the maximum RPM for both wheels
#define MAX_RPM                       120

// Set the minimum RPM for both wheels
#define MIN_RPM                       30

// Global variable used to store the current state of the bumper sensors when an interrupt
// occurs (Bumper_Sensors_Handler). It will get updated on each interrupt event.
uint8_t bumper_sensor_value         = 0;

// Global variable that gets set in Bumper_Sensors_Handler.
// This is used to detect if any collisions occurred
uint8_t collision_detected          = 0;

// Desired RPM for the left wheel
uint16_t Desired_RPM_Left           = 50;

// Desired RPM for the right wheel
uint16_t Desired_RPM_Right          = 50;

// Declare a global variable used to store the measured RPM by the left tachometer
uint16_t Actual_RPM_Left            = 0;

// Declare a global variable used to store the measured RPM by the right tachometer
uint16_t Actual_RPM_Right           = 0;

// Set initial duty cycle of the left wheel to 25%
uint16_t Duty_Cycle_Left            = 3750;

// Set initial duty cycle of the right wheel to 25%
uint16_t Duty_Cycle_Right           = 3750;

// Number of left wheel steps measured by the tachometer
int32_t Left_Steps                  = 0;

// Number of right wheel steps measured by the tachometer
int32_t Right_Steps                 = 0;

// Store tachometer period of the left wheel in a buffer
// Number of 83.3 ns clock cycles to rotate 1/360 of a wheel rotation
uint16_t Tachometer_Buffer_Left[BUFFER_LENGTH];

// Store tachometer period of the right wheel in a buffer
// Number of 83.3 ns clock cycles to rotate 1/360 of a wheel rotation
uint16_t Tachometer_Buffer_Right[BUFFER_LENGTH];

// Direction of the left wheel's rotation
enum Tachometer_Direction Left_Direction;

// Direction of the right wheel's rotation
enum Tachometer_Direction Right_Direction;

// Global variable used to keep track of the number of edges
uint16_t Edge_Counter               = 0;


/**
 * @brief Bumper sensor interrupt handler function.
 *
 * This is the interrupt handler for the bumper sensor interrupts. It is called when a falling edge event is detected on
 * any of the bumper sensor pins. The function checks if a collision has already been detected; if not, it prints a collision
 * detection message along with the bumper sensor state and sets a collision flag to prevent further detections.
 *
 * @param bumper_sensor_state An 8-bit unsigned integer representing the bump sensor states at the time of the interrupt.
 *
 * @return None
 */
void Bumper_Sensors_Handler(uint8_t bumper_sensor_state)
{
    if (collision_detected == 0)
    {
        printf("Collision Detected! Bumper Sensor State: 0x%02X\n", bumper_sensor_state);
        collision_detected = 1;
    }
}

// Global variable used for Task 1
uint8_t Done = 0;

/**
 * @brief Toggles an output pin until eight edges are counted
 *
 * This function checks an Edge Counter every time the timer A1 interrupt is fired (at 2 kHz).
 * If the Edge Count is less than 8, the pin P8.0 is toggle and the "Done" flag is cleared.
 * Otherwise, the functions sets a flag "Done" and clears the P8.0 pin. *
 *
 * @return None
 */
void Timer_A1_Periodic_Task(void)
{
    if (Edge_Counter < 8){
        P8->OUT ^= 1;
        Done = 0;
    }
    else{
        P8->OUT = 0;
        Done = 1;
    }
}

/**
 * @brief Timer A2 Capture task incrementing Edge Counter
 *
 * This function is called by the Timer A2 Capture interrupt handler, which is fired on the rising edge of pin P8.0.
 * The function increments the Edge Counter and sets the RGB LED to blue.
 *
 * @return None
 */
void Detect_Edge(uint16_t time)
{
    Edge_Counter++;
    LED2_Output(RGB_LED_BLUE);
}

/**
 * @brief Handles collision events.
 *
 * This function handles collision events by performing the following actions:
 * 1. Stops the motors to halt the robot's movement.
 * 2. Flashes LEDs to indicate a collision.
 * 3. Moves the motors backward to recover from the collision.
 * 4. Makes the robot turn right.
 * 5. Resets the collision detection flag.
 *
 * @return None
 */
void Handle_Collision()
{
    // Stop the motors
    Motor_Stop();
    // Make a function call to Clock_Delay1ms(2000)
    Clock_Delay1ms(2000);

    // Flash the LEDs to indicate collision event after stopping the motors
    for (int i = 0; i < 5; i++)
    {
        LED1_Output(RED_LED_OFF);
        LED2_Output(RGB_LED_OFF);
        Clock_Delay1ms(200);
        LED1_Output(RED_LED_ON);
        LED2_Output(RGB_LED_RED);
        Clock_Delay1ms(200);
    }

    // Move the motors backward with 30% duty cycle
    Motor_Backward(4500, 4500);
    // Make a function call to Clock_Delay1ms(3000)
    Clock_Delay1ms(3000);

    // Stop the motors
    Motor_Stop();
    // Make a function call to Clock_Delay1ms(1000)
    Clock_Delay1ms(1000);

    // Make the robot turn to the right with 10% duty cycle
    Motor_Right(1500, 1500);
    // Make a function call to Clock_Delay1ms(5000)
    Clock_Delay1ms(5000);

    // Stop the motors
    Motor_Stop();
    // Make a function call to Clock_Delay1ms(2000)
    Clock_Delay1ms(2000);

    // Set the collision_detected flag to 0
    collision_detected = 0;
}

/**
 * @brief Update desired RPM based on button presses.
 *
 * This function updates the desired RPM (revolutions per minute) for the robot's left and right motors
 * based on button presses. It checks the status of two buttons and increases the desired RPM by 10
 * if the corresponding button is pressed. If the desired RPM exceeds a maximum threshold, it wraps around
 * to the minimum RPM value.
 *
 * @return None
 */
void Update_Desired_RPM()
{
    uint8_t button_status = Get_Buttons_Status();

    // If Button 1 has been pressed
    if (button_status == 0x10)
    {
        Desired_RPM_Right = Desired_RPM_Right + 10;
        if (Desired_RPM_Right > MAX_RPM)
        {
            Desired_RPM_Right = MIN_RPM;
        }
    }

    // If Button 2 has been pressed
    if (button_status == 0x02)
    {
        Desired_RPM_Left = Desired_RPM_Left + 10;
        if (Desired_RPM_Left > MAX_RPM)
        {
            Desired_RPM_Left = MIN_RPM;
        }
    }
}

/**
 * @brief Calculate the average of an unsigned integer buffer.
 *
 * This function calculates the average value of an array of unsigned integer values.
 * It takes a pointer to the buffer and the length of the buffer as input, and it returns
 * the calculated average as a 16-bit unsigned integer.
 *
 * @param buffer A pointer to the array of unsigned integer values.
 * @param buffer_length The length of the buffer.
 *
 * @return The average value of the elements in the buffer as a uint16_t.
 */
uint16_t Average_of_Buffer(uint16_t *buffer, int buffer_length)
{
    uint32_t buffer_sum = 0;
    uint32_t buffer_average = 0;

    for (int i = 0; i < buffer_length; i++)
    {
        buffer_sum = buffer_sum + buffer[i];
    }

    buffer_average = buffer_sum / buffer_length;
    return buffer_average;
}

/**
 * @brief Function to drive the robot forward
 *
 * This is a helper function to drive the robot forward by setting similar PWM duty cycle for the right and the left motors.
 * Although in theory both duty cycles should be the same, the discrepancy in number is due to physical imperfections.
 * The method of trial and error is used to generate these numbers based on the specific robot used.
 *
 * @return None
 */
void go_straight () {
    Motor_Forward(4400, 4500);
}

/**
 * @brief Function to make the robot turn left
 *
 * This is a helper function to drive the robot left by setting high duty cycle to the right motor the the left motor.
 * The method of trial and error is used to generate these numbers based on the specific robot used and the test path.
 *
 * @return None
 */
void turn() {
    Motor_Forward(3500, 4500);
}

/**
 * @brief Function to make the robot complete a semi-circular path
 *
 * This function uses the helper functions go_straight() and turn() to drive the robot through a circular track.
 * The delays used in the function are based on the track itself and are set using trial and error method.
 * The function uses the red RGB LED to indicate that the robot hasn't reached the end yet. When the end is reached, the green RGB LED is used to indicate that.

 *
 * @return None
 */
void task2() {
    Clock_Delay1ms(1000);
    LED2_Output(RGB_LED_RED);
    go_straight();
    Clock_Delay1ms(6000);

    turn();
    Clock_Delay1ms(3000);

    go_straight();
    Clock_Delay1ms(3000);

    turn();
    Clock_Delay1ms(4000);

    go_straight();
    Clock_Delay1ms(6000);

    LED2_Output(RGB_LED_GREEN);
}

int main(void)
{
    int i = 0;

    // Initialize collision_detected flag
    collision_detected = 0;

    // Initialize the 48 MHz Clock
    Clock_Init48MHz();

    // Initialize the built-in red LED
    LED1_Init();
    LED2_Init();

    // Initialize the front and back LEDs
    P8_Init();

    // Initialize the buttons
    Buttons_Init();

    // Initialize EUSCI_A0_UART
    EUSCI_A0_UART_Init_Printf();

    // Initialize the bumper sensors which will be used to generate external I/O-triggered interrupts
    Bumper_Sensors_Init(&Bumper_Sensors_Handler);

    // Initialize Timer A1 with interrupts enabled
    // Default frequency is set to 10 Hz
    Timer_A1_Interrupt_Init(&Timer_A1_Periodic_Task, TIMER_A1_INT_CCR0_VALUE);

    // Initialize Timer A2 in Capture mode
    Timer_A2_Capture_Init(&Detect_Edge);

    // Initialize the tachometers
    Tachometer_Init();

    // Initialize the motors
    Motor_Init();

    // Enable the interrupts used by the bumper sensors and Timer A1
    EnableInterrupts();

    while(1)
    {
#if defined MAIN_CONTROLLER
    #if defined TASK_1 || TASK_2
        #error "Only MAIN_CONTROLLER, TASK_1, or TASK_2 can be active at the same time."
    #endif

        // Ensure that the motors are not running at the beginning
        Motor_Stop();

        // The bumper switches will be used to initiate the start of the motors
        // If the bumper switches haven't been pressed, then the RPM can be updated
        // by pressing the user buttons
        while (collision_detected == 0)
        {
            LED1_Output(RED_LED_ON);
            LED2_Output(RGB_LED_RED);
            Update_Desired_RPM();
            printf("Desired_RPM_Left: %d | Desired_RPM_Right: %d\n", Desired_RPM_Left, Desired_RPM_Right);
            Clock_Delay1ms(200);
        }

        // Flash the LEDs to indicate exit from while-loop (i.e. collision event detected)
        for (int i = 0; i < 5; i++)
        {
            LED1_Output(RED_LED_OFF);
            LED2_Output(RGB_LED_OFF);
            Clock_Delay1ms(200);
            LED1_Output(RED_LED_ON);
            LED2_Output(RGB_LED_GREEN);
            Clock_Delay1ms(200);
        }

        // After exiting the while-loop, Bumper_Sensors_Handler sets collision_detected to 1
        // Clear the collision_detected flag here
        collision_detected = 0;

        // Ensure that i is 0
        i = 0;

        // Calculate the actual RPM and move the motors forward with updated duty cycle values
        // if there is no collision event detected
        while (collision_detected == 0)
        {
            // Get the measurements made by the tachometers
            Tachometer_Get(&Tachometer_Buffer_Left[i], &Left_Direction, &Left_Steps, &Tachometer_Buffer_Right[i], &Right_Direction, &Right_Steps);
            i = i + 1;
            if (i >= BUFFER_LENGTH)
            {
                i = 0;

                // (1/Tachometer Step/Cycles) * (12,000,000 Cycles / Second) * (60 Second / Minute) * (1/360 Rotation/Step)
                Actual_RPM_Left = 2000000 / (Average_of_Buffer(Tachometer_Buffer_Left, BUFFER_LENGTH));
                Actual_RPM_Right = 2000000 / (Average_of_Buffer(Tachometer_Buffer_Right, BUFFER_LENGTH));

                // If the actual RPM measured on the left wheel is greater than the desired RPM,
                // then decrease the duty cycle on the left wheel
                if ((Actual_RPM_Left > (Desired_RPM_Left + 3)) && (Duty_Cycle_Left > 100))
                {
                    Duty_Cycle_Left = Duty_Cycle_Left - 100;
                }

                // Otherwise, if the actual RPM is less than the desired RPM,
                // then increase the duty cycle on the left wheel
                else if ((Actual_RPM_Left < (Desired_RPM_Left - 3)) && (Duty_Cycle_Left < 14898))
                {
                    Duty_Cycle_Left = Duty_Cycle_Left + 100;
                }

                // If the actual RPM measured on the right wheel is greater than the desired RPM,
                // then decrease the duty cycle on the right wheel
                if ((Actual_RPM_Right > (Desired_RPM_Right + 3)) && (Duty_Cycle_Right > 100))
                {
                    Duty_Cycle_Right = Duty_Cycle_Right - 100;
                }

                // Otherwise, if the actual RPM is less than the desired RPM,
                // then increase the duty cycle on the right wheel
                else if ((Actual_RPM_Right < (Desired_RPM_Right - 3)) && (Duty_Cycle_Right < 14898))
                {
                    Duty_Cycle_Right = Duty_Cycle_Right + 100;
                }

                // Move the motors forward with the updated duty cycle
                Motor_Forward(Duty_Cycle_Left, Duty_Cycle_Right);

                // Compare the desired RPM and actual RPM values using the serial terminal
                printf("Desired_RPM_Left: %d | Desired_RPM_Right: %d\n", Desired_RPM_Left, Desired_RPM_Right);
                printf("Actual_RPM_Left: %d | Actual_RPM_Right: %d\n", Actual_RPM_Left, Actual_RPM_Right);
                printf("Left_Steps: %d | Right Steps: %d\n\n", Left_Steps, Right_Steps);
            }
            Clock_Delay1ms(100);
        }

        // Handle the collision event
        while (collision_detected == 1)
        {
            Handle_Collision();
        }

#elif defined TASK_1
    #if defined MAIN_CONTROLLER || TASK_2
        #error "Only MAIN_CONTROLLER, TASK_1, or TASK_2 can be active at the same time."
    #endif

        uint8_t button_status = Get_Buttons_Status();
        if (button_status == 0x10){
            Edge_Counter = 0;
            Done = 0;
            printf("RESETTING!");
        }
        if (Done == 1)
        {
            printf("Edge Counter: %d\n", Edge_Counter);
            Clock_Delay1ms(200);
        }

#elif defined TASK_2
    #if defined MAIN_CONTROLLER || TASK_1
        #error "Only MAIN_CONTROLLER, TASK_1, or TASK_2 can be active at the same time."
    #endif

        task2();
        Motor_Stop();
        break;

#else
    #error "Define either one of the options: MAIN_CONTROLLER, TASK_1, or TASK_2."
#endif
    }
}
