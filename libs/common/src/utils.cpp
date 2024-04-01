#include "utils.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "drivetrain_config.h"

volatile bool ESCdelayInProgress = false;
volatile bool ESCirqTriggered = false;

void initI2C(i2c_inst_t* &i2c_port, uint baudrate, uint sda_pin, uint scl_pin) {
    i2c_port = i2c_default; // or i2c0, i2c1, etc.
    i2c_init(i2c_port, baudrate);

    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
}

float wrap_pi(const float heading) {
    // constrain heading to within +/-pi (+/-180 degrees) without changing the meaning of the angle
    // if its more than pi (+180), subtract 2*pi (subtract 360degrees) so we have the "smaller" angle 
    float wrapped = heading;

    if (heading > M_PI) {
        wrapped = heading - M_TWOPI;
    } else if (heading < -M_PI) {
        wrapped = heading + M_TWOPI;
    }

    return static_cast<float>(wrapped);
}

void initMotorMonitorPins() {
    using namespace CONFIG;
    
    // Configure motorSleepPin as output and set it to high
    gpio_init(motorSleepPin);
    gpio_set_dir(motorSleepPin, GPIO_OUT);
    gpio_put(motorSleepPin, true);

    // Configure motorStatusPin as input
    gpio_init(motorStatusPin);
    gpio_set_dir(motorStatusPin, GPIO_IN);
    printf("motor pins initialised\n");
}

void handlerMotorController(uint gpio, uint32_t events) {
    ESCirqTriggered = true;
}

// Non-blocking delay function
bool non_blocking_delay_us(uint32_t delay_us) {
    static absolute_time_t start_time;
    static bool delay_started = false;

    if (!delay_started) {
        start_time = get_absolute_time();
        delay_started = true;
    }

    if (absolute_time_diff_us(start_time, get_absolute_time()) >= delay_us) {
        delay_started = false; // Reset for the next delay call
        return true; // Delay completed
    }

    return false; // Delay still ongoing
}

void toggleMotorSleepPin(){
    gpio_put(CONFIG::motorSleepPin, false);
    sleep_us(15);
    gpio_put(CONFIG::motorSleepPin, true);
    sleep_us(10);
}