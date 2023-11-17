#include <cstdio>
#include "pico/stdlib.h"

#include "motor2040.hpp"
#include "button.hpp"
#include "pid.hpp"

/*
A program to aid in the discovery and tuning of motor PID
values for velocity control. It does this by commanding the
motor to drive repeatedly between two setpoint speeds and
plots the measured response.

Press "Boot" to exit the program.
*/

using namespace motor;
using namespace encoder;



// The gear ratio of the motor
constexpr float GEAR_RATIO = 19.22f;

constexpr int CPR = 12;

// The counts per revolution of the motor's output shaft
constexpr float COUNTS_PER_REV = CPR * GEAR_RATIO;

// The direction to spin the motor in. NORMAL_DIR (0), REVERSED_DIR (1)
const Direction DIRECTION = NORMAL_DIR;

// The scaling to apply to the motor's speed to match its real-world speed
constexpr float SPEED_SCALE = 495.0f;

// How many times to update the motor per second
const uint UPDATES = 100;
constexpr float UPDATE_RATE = 1.0f / (float)UPDATES;

// The time (in milliseconds) it approximately takes for the script to run, inc prints
constexpr uint32_t UPDATE_TIME = 6;

// The time (in seconds) between each new setpoint being set
constexpr float MOVEMENT_WINDOW = 1.0f;

// The time (in seconds) after a new setpoint, to display print out motor values
constexpr float PRINT_WINDOW = MOVEMENT_WINDOW;

// How many of the updates should be printed (i.e. 2 would be every other update)
const uint PRINT_DIVIDER = 1;

// Multipliers for the different printed values, so they appear nicely on the Thonny plotter
constexpr float ACC_PRINT_SCALE = 1.0f;    // Acceleration multiplier

// How far from zero to drive the motor at, in revolutions per second
constexpr float VELOCITY_EXTENT = 300.0f;

// Motor and Encoder pins for each motor
Motor motors[] = {
    Motor(motor2040::MOTOR_A, DIRECTION, SPEED_SCALE),
    Motor(motor2040::MOTOR_B, DIRECTION, SPEED_SCALE),
    Motor(motor2040::MOTOR_C, DIRECTION, SPEED_SCALE),
    Motor(motor2040::MOTOR_D, DIRECTION, SPEED_SCALE)
};

Encoder encoders[] = {
    Encoder(pio0, 0, motor2040::ENCODER_A, PIN_UNUSED, DIRECTION, COUNTS_PER_REV, true),
    Encoder(pio0, 1, motor2040::ENCODER_B, PIN_UNUSED, DIRECTION, COUNTS_PER_REV, true),
    Encoder(pio0, 2, motor2040::ENCODER_C, PIN_UNUSED, DIRECTION, COUNTS_PER_REV, true),
    Encoder(pio0, 3, motor2040::ENCODER_D, PIN_UNUSED, DIRECTION, COUNTS_PER_REV, true)
};

// PID values
constexpr float VEL_KP = 3.25f;   // Velocity proportional (P) gain
constexpr float VEL_KI = 0.0f;    // Velocity integral (I) gain
constexpr float VEL_KD = 0.003f;    // Velocity derivative (D) gain


// Create the user button
Button user_sw(motor2040::USER_SW);

// PID objects for each motor
PID vel_pids[4];


int main() {
  stdio_init_all();

  // Initialize motors, encoders, and PIDs
  for (int i = 0; i < 4; ++i) {
      vel_pids[i] = PID(VEL_KP, VEL_KI, VEL_KD, UPDATE_RATE);
      motors[i].init();
      encoders[i].init();
      motors[i].enable();
  }

  // direction mapping for the four wheels
  vel_pids[0].setpoint = VELOCITY_EXTENT;
  vel_pids[1].setpoint = -VELOCITY_EXTENT;
  vel_pids[2].setpoint = VELOCITY_EXTENT;
  vel_pids[3].setpoint = -VELOCITY_EXTENT;

  uint update = 0;
  uint print_count = 0;

  // Continually move the motor until the user button is pressed
  while(!user_sw.raw()) {

    for (int i = 0; i < 4; ++i) {
        Encoder::Capture capture = encoders[i].capture();
        float accel = vel_pids[i].calculate(capture.revolutions_per_minute());
        motors[i].speed(vel_pids[i].setpoint + accel);

        if (update < (uint)(PRINT_WINDOW * UPDATES) && print_count == 0 && i==0) {
            printf("Motor %d, Time = %.4f, Vel = %f, Vel SP = %f, Accel = %f, Speed = %f\n",
                    i, to_ms_since_boot(get_absolute_time())/1000.0,
                    capture.revolutions_per_minute(), vel_pids[i].setpoint,
                    accel * ACC_PRINT_SCALE, motors[i].speed());
        }
    }

    // Increment the print count, and wrap it
    print_count = (print_count + 1) % PRINT_DIVIDER;

    update++;   // Move along in time

    // Have we reached the end of this time window?
    if (update >= (uint)(MOVEMENT_WINDOW * UPDATES)) {
        update = 0;
        for (int i = 0; i < 4; ++i) {
            vel_pids[i].setpoint = 0.0f - vel_pids[i].setpoint;
        }
    }

    sleep_ms(UPDATE_RATE * 1000.0f - UPDATE_TIME);
  }

  // Disable the motor
  for (int i = 0; i < 4; ++i) {
      motors[i].disable();
  }
}
