#include <cstdio>
#include "pico/stdlib.h"

#include "motor2040.hpp"

/*
A program that profiles the speed of a motor across its PWM
duty cycle range using the attached encoder for feedback.
*/

using namespace motor;
using namespace encoder;


// Constants
constexpr float GEAR_RATIO = 19.22;
constexpr int CPR = 12;
constexpr float COUNTS_PER_REV = CPR * GEAR_RATIO;
constexpr float SPEED_SCALE = 495;
constexpr float ZERO_POINT = 0.0f;
constexpr float DEAD_ZONE = 0.09f;
const uint DUTY_STEPS = 100;
const uint SETTLE_TIME = 100;
const uint CAPTURE_TIME = 200;
const Direction DIRECTION = NORMAL_DIR;

// Motor and Encoder Objects
Motor motors[] = {
    Motor(motor2040::MOTOR_A, DIRECTION, SPEED_SCALE, ZERO_POINT, DEAD_ZONE),
    Motor(motor2040::MOTOR_B, DIRECTION, SPEED_SCALE, ZERO_POINT, DEAD_ZONE),
    Motor(motor2040::MOTOR_C, DIRECTION, SPEED_SCALE, ZERO_POINT, DEAD_ZONE),
    Motor(motor2040::MOTOR_D, DIRECTION, SPEED_SCALE, ZERO_POINT, DEAD_ZONE)
};

Encoder encoders[] = {
    Encoder(pio0, 0, motor2040::ENCODER_A, PIN_UNUSED, DIRECTION, COUNTS_PER_REV, true),
    Encoder(pio0, 1, motor2040::ENCODER_B, PIN_UNUSED, DIRECTION, COUNTS_PER_REV, true),
    Encoder(pio0, 2, motor2040::ENCODER_C, PIN_UNUSED, DIRECTION, COUNTS_PER_REV, true),
    Encoder(pio0, 3, motor2040::ENCODER_D, PIN_UNUSED, DIRECTION, COUNTS_PER_REV, true)
};

// Additional variables to track the minimum duty cycle required to start movement
float min_positive_duty = 1.0f; // Start with the maximum duty
float min_negative_duty = -1.0f; // Start with the minimum negative duty
bool found_positive_start = false;
bool found_negative_start = false;

// Function to profile a motor at a given duty
void profile_at_duty(Motor motors[], Encoder encoders[], float duty, float min_speeds[], 
                     float max_speeds[], bool found_pos_starts[], float min_pos_duties[],
                     bool found_neg_starts[], float min_neg_duties[]) {

// Set duty for all motors
  for(int i = 0; i < 4; i++) {
    motors[i].duty((DIRECTION == REVERSED_DIR) ? (0.0 - duty) : duty);
  }
  sleep_ms(SETTLE_TIME); // Wait for motors to settle

// Capture speeds for all motors
  for(int i = 0; i < 4; i++) {
    encoders[i].capture(); // Clear the encoder
  }

  // Wait for the capture time to pass
  sleep_ms(CAPTURE_TIME);

  for(int i = 0; i < 4; i++) {
  
    // Perform a capture and read the measured speed
    Encoder::Capture capture = encoders[i].capture();
    
    float measured_speed = capture.revolutions_per_minute();

    // These are some alternate speed measurements from the encoder
    //float measured_speed = capture.revolutions_per_second();
    // float measured_speed = capture.degrees_per_second();
    // float measured_speed = capture.radians_per_second();

    // Print out the expected and measured speeds, as well as their difference
    //printf("Duty = %f, Expected = %f, Measured = %f, Diff = %f\n",
    //       m.duty(), m.speed(), measured_speed, m.speed() - measured_speed);


    // Update the maximum and minimum speeds
    if (measured_speed > max_speeds[i]) {
      max_speeds[i] = measured_speed;
    } else if (measured_speed < min_speeds[i]) {
      min_speeds[i] = measured_speed;
    }

    // Check for start of movement
    if (!found_pos_starts[i] && (duty > 0 && measured_speed > 0)) {
      min_pos_duties[i] = duty;
      found_pos_starts[i] = true;
    }
    if (!found_neg_starts[i] && duty < 0 && measured_speed < 0) {
      min_neg_duties[i] = duty;
      found_neg_starts[i] = true;
    }
  }
}


int main() {
  stdio_init_all();

  // Give some time to connect up a serial terminal
  sleep_ms(10000);

  float min_speeds[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float max_speeds[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  bool found_pos_starts[4] = {false, false, false, false};
  float min_pos_duties[4] = {1.0f, 1.0f, 1.0f, 1.0f};
  bool found_neg_starts[4] = {false, false, false, false};
  float min_neg_duties[4] = {1.0f, 1.0f, 1.0f, 1.0f};

  // Initialize motors and encoders, and maximum speed array
  for(int i = 0; i < 4; i++) {
      motors[i].init();
      motors[i].enable();
      encoders[i].init();
  }

  printf("Profiler Starting...\n");

    // Profile from 0% up to one step below 100%
  for(uint j = 0; j < DUTY_STEPS; j++) {
    profile_at_duty(motors, encoders, (float)j / (float)DUTY_STEPS,
                    min_speeds, max_speeds, found_pos_starts,
                    min_pos_duties, found_neg_starts, min_neg_duties);
  }
  // Profile from 100% down to one step above 0%
  for(uint j = 0; j < DUTY_STEPS; j++) {
    profile_at_duty(motors, encoders, (float)(DUTY_STEPS - j) / (float)DUTY_STEPS,
                    min_speeds, max_speeds, found_pos_starts,
                    min_pos_duties, found_neg_starts, min_neg_duties);
  }
  // Profile from 0% down to one step above -100%
  for(uint j = 0; j < DUTY_STEPS; j++) {
    profile_at_duty(motors, encoders, -(float)j / (float)DUTY_STEPS,
                    min_speeds, max_speeds, found_pos_starts,
                    min_pos_duties, found_neg_starts, min_neg_duties);
  }
  // Profile from -100% up to one step below 0%
  for(uint j = 0; j < DUTY_STEPS; j++) {
    profile_at_duty(motors, encoders, -(float)(DUTY_STEPS - j) / (float)DUTY_STEPS,
                    min_speeds, max_speeds, found_pos_starts,
                    min_pos_duties, found_neg_starts, min_neg_duties);
  }
  // Profile 0% again
  profile_at_duty(motors, encoders, 0.0f, min_speeds,
                  max_speeds, found_pos_starts, min_pos_duties,
                  found_neg_starts, min_neg_duties);

  for(int i = 0; i < 4; i++) {
      motors[i].disable();
  }

  for(int i = 0; i < 4; i++) {
    printf("Motor %d - Min Speed: %f, Max Speed: %f, Min neg Duty: %f, Min Pos Duty: %f\n", 
           i, min_speeds[i], max_speeds[i], min_neg_duties[i], min_pos_duties[i]);
  }

  printf("Profiler Finished...\n");

}
