//
// Created by robbe on 01/10/2023.
//

#ifndef OSOD_MOTOR_2040_ACKERMANN_MIXER_H
#define OSOD_MOTOR_2040_ACKERMANN_MIXER_H


struct MotorSpeed4wd {
    float frontleft;
    float frontright;
    float rearleft;
    float rearright;
};

class AckermannMixer {
private:
    float wheelTrack;         // Distance between the left and right wheels (mm)
    float wheelBase;          // Distance between the front and rear wheels (mm)
    float maxSteeringAngle;   // Max angle from straight ahead that the steerable wheels can pivot (degrees)

public:
    AckermannMixer(float track, float base, float angle); // constructor
    struct MotorSpeed4wd ackermann_steer_mix(float yaw, float throttle); //mixing function

};
#endif //OSOD_MOTOR_2040_ACKERMANN_MIXER_H
