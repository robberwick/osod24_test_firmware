// drivetrain_config.h
#ifndef DRIVETRAIN_CONFIG_H
#define DRIVETRAIN_CONFIG_H

enum Challenge {
    LAVA_PALAVA,
    ECO_DISASTER,
    ESCAPE_ROUTE,
    MINESWEEPER,
    ZOMBIE_APOCALYPSE,
    PI_NOON,
    TEMPLE_OF_DOOM
};
// define CURRENT_CHALLENGE in a earlier file with:
//Challenge CURRENT_CHALLENGE = LAVA_PALAVA;
// To set the current challenge and the drivetrain config appropriately

// geometry
const float MAX_STEERING_ANGLE = 60; //degrees
const float WHEEL_BASE = 0.18; //metres
const float WHEEL_TRACK = 0.15; //metres

const float LARGE_WHEEL_DIAMETER = 0.0816; //metres valid for Lego 2902 "81.6" tyres
const float SMALL_WHEEL_DIAMETER = 0.0495; //metres valid for Lego 15413 tyres
const float MECANUM_DIAMETER = 0.048; //metres, valid for "48mm" mecanums

// Define WHEEL_DIAMETER and GEAR_RATIO based on CURRENT_CHALLENGE
#if CURRENT_CHALLENGE == ECO_DISASTER
    const float WHEEL_DIAMETER = LARGE_WHEEL_DIAMETER;
    const float GEAR_RATIO = 42/18;
#elif CURRENT_CHALLENGE == ESCAPE_ROUTE || CURRENT_CHALLENGE == MINESWEEPER || CURRENT_CHALLENGE == ZOMBIE_APOCALYPSE
    const float WHEEL_DIAMETER = SMALL_WHEEL_DIAMETER;
    const float GEAR_RATIO = 1;
#elif CURRENT_CHALLENGE == PI_NOON
    const float WHEEL_DIAMETER = MECANUM_DIAMETER;
    const float GEAR_RATIO = 1;
#else  // LAVA_PALAVA, TEMPLE_OF_DOOM
    // Default case
    const float WHEEL_DIAMETER = LARGE_WHEEL_DIAMETER;
    const float GEAR_RATIO = 1;
#endif

//dynamics
const float MAX_VELOCITY = 1; // m/s
const float MAX_ACCELERATION = 8; // m/s^2
const float MAX_ANGULAR_VELOCITY = 5; // rad/s
const float MAX_ANGULAR_ACCELERATION = 20; // rad/s^2

#endif // DRIVETRAIN_CONFIG_H

