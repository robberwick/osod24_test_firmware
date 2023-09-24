# Architecture

The high level architecture of the firmware comprises four main components:

- [**Drive Train Config**](#drive-train-config): This component is responsible for configuring the drivetrain.
- [**Navigator**](#navigator): This component is responsible for determining the robot's desired state. It takes in a state request and outputs a desired state.
- [**State Estimator**](#state-estimator): This component is responsible for estimating the state of the robot. It takes in raw sensor data
and outputs a filtered state estimate. The state estimate is used by the Navigator to determine the robot's position and orientation.
- [**State Manager**](#state-manager): This component is responsible for controlling the robot. It takes in a state request, as well as 
current state, and enacts the changes necessary to try to achieve the desired state and outputs a control request.

Not part of the firmware, but an important part of the overall system, is the [**Pi**](#pi).

The relationship between these components is shown in the [block diagram](#block-diagram) below.

More detailed information about each component can be found in the following sections.

## Pi

The Pi is responsible for sending high level path planning requests to the robot, and receiving telemetry data from the robot.

## Drive Train Config

The Drive Train Config component is responsible for configuring the drivetrain. It provides values capturing the physical
attributes of the robot which are required by various algorithms in the firmware, such as the wheelbase and wheel diameter.
It can also provide optional parameters such as the maximum velocity, maximum acceleration, and maximum angular velocity.

## Navigator

The Navigator component is responsible for directing changes to the robot's state. It takes the current robot state as input,
and outputs a state request which is used by the State Manager to try to achieve the desired state.

The Navigator is responsible for receiving high level path planning requests from the Pi, and converting them into a series
of state requests which are sent to the State Manager. It is also responsible for relaying telemetry data from the robot
to the Pi.

The navigator also contains a control mux which is responsible for switching control mode between the Pi and an RC 
transmitter, connected via a Receiver (Rx). It does this by maintaining an always on connection with the Rx, and switching
between the Pi and the RX based on the state of one of the channels of the Rx i.e. an AUX switch. This enabled autonomous
and manual control of the robot.

## State Estimator

The State Estimator component is responsible for estimating the state of the robot. It gathers raw sensor data e.g. IMU and encoders 
and outputs the following:
- Sensor state ('raw state') which contains low level sensor data such as encoder counts and IMU readings.
- Robot state ('filtered state') which contains the estimated state of the robot e.g. position and orientation.

These values are used by the Navigator and the State Manager.

## State Manager

The State Manager component is responsible for controlling the robot. It takes in a state request, as well as the current
state of the robot, and coordinates the changes necessary to try to achieve the desired state. It comprises the following
subcomponents:

- [**V PID**](#v-pid)
- [**W PID**](#w-pid)
- [**Mixer**](#mixer) 
- [**Servo Manager**](#servo-manager)
- [**Stoker**](#stoker)

### V PID
This component is responsible for achieving and maintaining the desired linear velocity of the robot. 

### W PID
This component is responsible for achieving and maintaining the desired angular velocity of the robot.

### Mixer
This component is responsible for converting the output of the V PID and W PID into the desired RPM values for the motors,
and the desired servo angles to achieve the desired linear and angular velocities. There will be multiple mixer implementations
for different drive trains e.g. tank drive, mechanum drive, ackermann drive.

### Servo Manager
This component is responsible for controlling the servos necessary for steering.

### Stoker
This component is responsible for controlling the motors necessary for driving. It contains a PID controller for each motor, which is 
responsible for achieving and maintaining the desired RPM of the motor. There is one Stoker for each motor.

## Block Diagram

```text
                                                ┌───────────────┐
                                                │StateEstimator │
                             Filtered           │               │
                     ┌──────────────────────────┤ IMU           │
                     │ RobotState               │ Encoders      │
                     │                          │ ...           │
                     │                          └──────┬────────┘
                     │                                 │ Raw - RobotState & SensorState
                     │                                 ▼
                     │                          ┌──────────────────────────────────────────────────────────┐
                     ▼                          │StateManager                                              │
             ┌───────────────┐                  │                                                          │
             │ Navigator     │                  │ ┌────────────┐     ┌──────┐ ┌──────┐                     │
             │               │                  │ │ServoManager│     │V PID │ │W PID │                     │
             │  ┌──┐         │                  │ │            │     └──┬───┘ └───┬──┘                     │
             │  │Rx│         │  StateRequest    │ │            │        │         │                        │
             │  └─┬┘         │  V + W           │ │  ┌──────┐  │        ▼         ▼                        │
             │    │          │                  │ │  │Servo │  │     ┌───────────────┐                     │
             │    ▼          ├─────────────────►│ │  └──────┘  │     │               ├──┐                  │
┌──┐         │  ┌──────────┐ │                  │ │            │◄────┤    Mixer      │  │                  │
│Pi├─────────┤► │ControlMux│ │                  │ │  ┌──────┐  │     │               │  │                  │
└──┘         │  └──────────┘ │                  │ │  │Servo │  │     └─┬─────────────┘  │                  │
             └───────────────┘                  │ │  └──────┘  │       └──────┬─────────┘                  │
                                                │ └────────────┘              │                            │
                     ┌────────────────┐         │              ┌─────────┬────┴──────┬────────────┐        │
                     │DrivetrainConfig├────────►│              │         │           │            │        │
                     └────────────────┘         │              ▼         ▼           ▼            ▼        │
                                                │      ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐  │
                                                │      │Stoker   │  │Stoker   │  │Stoker   │  │Stoker   │  │
                                                │      │         │  │         │  │         │  │         │  │
                                                │      │ RPM PID │  │ RPM PID │  │ RPM PID │  │ RPM PID │  │
                                                │      └─────────┘  └─────────┘  └─────────┘  └─────────┘  │
                                                └──────────────────────────────────────────────────────────┘
```
[link](https://asciiflow.com/#/share/eJzVlr1OwzAQgF%2FF8sBC6YBAgq5QJIagKhVlyWLAVBEhFq5bUqFKCDEyMESlz1LxNHkS0rRB%2BfFvmrYiuiE%2BO3ff%2FdjOK%2FTRE4Ytf%2Bh5DeihMaawBV8dGDiwdXp82nDgOH47PDmK3xgOWDxwIDB8ovDHVBzHr%2BClyxDD7QFznxAjFGjYuXA9him%2Bz9spWxbZqRCaUsClda3tHdjklrAkcM5k278j95gOtCwJn8Vks9nUZ5JZqqsTpH7WxMyssdELOMgmeQ90sT8gNBntEm66tRLsXISBTuWBJjWykI%2F6mCoTWvy46LTKTubZNQNRQ4ErNHL7yYEn9LBQGmKLa0Gj8DMK30ryZQJd5hNBdzEdkVURM3A90Lk8BynQ%2B81q%2BB4PTTC45ZLh8LCFiYzCMM3OUrV8n5lD2kEJMOluGz8P8YApAAsxSbrQPE89sA9ueHkStcki%2BCxYZhNLNrS4j3LRyEsX90fSTmmjqApotLflkNN5TldFpvNMICEvt7NcT5bh5CdTjC4us1ZODDZO9P3ByajlBulRnXqVoHZcnaQt1p8Rn1HiWcNAjanTt9zsqnA3l9la%2B1oSRDlihSYPqepZrgUDHPNYhbA8%2FxU86MJrPJH5Hc6JQWS6ItAa8xKnayaaF%2F7WRC%2B8c%2BqOMKPI9eOT6cHty2wuj31havOTwubdar6FV3rhgs8M1%2BxBDVrlktXJWcOiOsLpMvKY3IWpvaxiRaFQ1YGRVxYUhV85gaoWDLtjpX%2F8JUXqU67aWo%2BV77g%2FqXPJBo5s1cfVXP47ceAETn4B17mYzw%3D%3D)