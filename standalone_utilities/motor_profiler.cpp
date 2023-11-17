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
void profile_at_duty(Motor& m, Encoder& enc, float duty, float& min_speed, 
                     float& max_speed, bool& found_positive_start,
                     bool& found_negative_start, float& min_positive_duty, float& min_negative_duty) {

  // Set the motor to a new duty cycle and wait for it to settle
  if(DIRECTION == REVERSED_DIR)
      m.duty(0.0 - duty);
  else
      m.duty(duty);
  sleep_ms(SETTLE_TIME);

  // Perform a dummy capture to clear the encoder
  enc.capture();

  // Wait for the capture time to pass
  sleep_ms(CAPTURE_TIME);

  // Perform a capture and read the measured speed
  Encoder::Capture capture = enc.capture();
  //float measured_speed = capture.revolutions_per_second();

  // These are some alternate speed measurements from the encoder
  float measured_speed = capture.revolutions_per_minute();
  // float measured_speed = capture.degrees_per_second();
  // float measured_speed = capture.radians_per_second();

  // Print out the expected and measured speeds, as well as their difference
  //printf("Duty = %f, Expected = %f, Measured = %f, Diff = %f\n",
  //       m.duty(), m.speed(), measured_speed, m.speed() - measured_speed);
             // Update the maximum speed if a higher speed is measured
  if (measured_speed > max_speed) {
      max_speed = measured_speed;
  } else if (measured_speed < min_speed) {
      min_speed = measured_speed;
  }
  if (!found_positive_start && duty > 0 && measured_speed > 0) {
      min_positive_duty = duty;
      found_positive_start = true;
  }
  if (!found_negative_start && duty < 0 && measured_speed < 0) {
      min_negative_duty = duty;
      found_negative_start = true;
  }
}


int main() {
  stdio_init_all();

  // Give some time to connect up a serial terminal
  sleep_ms(10000);

  float min_speeds[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float max_speeds[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  // Initialize motors and encoders, and maximum speed array
  for(int i = 0; i < 4; i++) {
      motors[i].init();
      motors[i].enable();
      encoders[i].init();
  }

  printf("Profiler Starting...\n");

  // Perform profiling for each motor
  for(int i = 0; i < 4; i++) {
      found_positive_start = found_negative_start = false;
      min_positive_duty = 1.0f;
      min_negative_duty = -1.0f;
      
      printf("Profiling Motor %d...\n", i);
      // Profile from 0% up to one step below 100%
      for(uint j = 0; j < DUTY_STEPS; j++) {
        profile_at_duty(motors[i], encoders[i], (float)j / (float)DUTY_STEPS,
                        min_speeds[i], max_speeds[i], found_positive_start,
                        found_negative_start, min_positive_duty, min_negative_duty);
      }
      // Profile from 100% down to one step above 0%
      for(uint j = 0; j < DUTY_STEPS; j++) {
        profile_at_duty(motors[i], encoders[i], (float)(DUTY_STEPS - j) / (float)DUTY_STEPS,
                        min_speeds[i], max_speeds[i], found_positive_start,
                        found_negative_start, min_positive_duty, min_negative_duty);
      }
      // Profile from 0% down to one step above -100%
      for(uint j = 0; j < DUTY_STEPS; j++) {
        profile_at_duty(motors[i], encoders[i], -(float)j / (float)DUTY_STEPS,
                        min_speeds[i], max_speeds[i], found_positive_start,
                        found_negative_start, min_positive_duty, min_negative_duty);
      }
      // Profile from -100% up to one step below 0%
      for(uint j = 0; j < DUTY_STEPS; j++) {
        profile_at_duty(motors[i], encoders[i], -(float)(DUTY_STEPS - j) / (float)DUTY_STEPS,
                        min_speeds[i], max_speeds[i], found_positive_start,
                        found_negative_start, min_positive_duty, min_negative_duty);
      }
      // Profile 0% again
      profile_at_duty(motors[i], encoders[i], 0.0f, min_speeds[i],
                      max_speeds[i], found_positive_start, found_negative_start,
                      min_positive_duty, min_negative_duty);
      motors[i].disable();
      printf("Min and maximum speeds for Motor %d: %f, %f\n", i, min_speeds[i], max_speeds[i]);
      printf("Min Positive Duty = %f, Min Negative Duty = %f\n", i, min_positive_duty, min_negative_duty);
  }



  printf("Profiler Finished...\n");

}
