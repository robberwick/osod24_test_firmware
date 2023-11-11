//
// Created by markm on 11/11/2023.
//

#ifndef OSOD_MOTOR_2040_ACKERMANN_MIXER_H
#define OSOD_MOTOR_2040_ACKERMANN_MIXER_H


struct AckermannOutput {
    float frontLeftSpeed;
    float frontRightSpeed;
    float rearLeftSpeed;
    float rearRightSpeed;
    float frontLeftAngle;
    float frontRightAngle;
};

class AckermannMixer {
private:
    float wheelTrack;         // Distance between the left and right wheels (mm)
    float wheelBase;          // Distance between the front and rear wheels (mm)
    float maxSteeringAngle;   // Max angle from straight ahead that the steerable wheels can pivot (radians)
    float turnRadius;         // calculated turn radius from inputs, mm to centreline

public:
    AckermannMixer(float track, float base, float angle); // constructor
    float getTurnRadius() const; //getter for turn radius
    void setMaxSteeringAngle(float angle); //getter for turn radius
    struct AckermannOutput ackermann_steer_mix(float yaw, float throttle); //mixing function

};
#endif //OSOD_MOTOR_2040_ACKERMANN_MIXER_H
