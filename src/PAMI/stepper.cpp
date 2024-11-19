#include "pami_settings.h"
#include "pathing.h"
#include <math.h>
#include <stdbool.h>

#ifndef ENABLE_ARDUINO
    #include "idle_arduino.hpp"
    #include <unistd.h>
#else
    #include <Arduino.h>
    #include <FspTimer.h>
#endif

#define NUM_MOTORS 2
#define UPDATE_FREQ 3000.f                       // Hz
#define IDLE_PERIOD (1e6f / (UPDATE_FREQ * .9f)) // in microseconds
#define PATHING_UPDATE_PERIOD 10                 // in milliseconds

enum Side_t { LEFT = 0, RIGHT = 1 };

typedef struct {
    int ena;
    int dir;
    int pul;
} Pins;

typedef struct {
    enum Side_t side;
    float period;
    float speed;
    unsigned long nextTickTime;
    Pins pins;
    int pulValue;
    int ticksHistory;
    int rotationForward;
} Motor;

Motor motors[NUM_MOTORS];
FspTimer motorTimers;

void motor_on(bool on, int motorId) { digitalWrite(motors[motorId].pins.ena, on ? LOW : HIGH); }
void free_wheeling(int motorId) { motor_on(false, motorId); }
void move_clockwise(bool cw, int motorId) { digitalWrite(motors[motorId].pins.dir, cw ? LOW : HIGH); }

void motor_tick(const int motorId) {
    digitalWrite(motors[motorId].pins.pul, motors[motorId].pulValue);
    motors[motorId].pulValue = 1 - motors[motorId].pulValue;
    motors[motorId].nextTickTime += motors[motorId].period;
    motors[motorId].ticksHistory += motors[motorId].rotationForward;
}

void motors_tick(timer_callback_args_t __attribute((unused)) * p_args) {
    const unsigned long time = micros();
    for (int motorId = 0; motorId < NUM_MOTORS; motorId++) {
        if (motors[motorId].nextTickTime <= time) {
            motor_tick(motorId);
        }
    }
}

bool begin_timer(const float rate) {
    int8_t timer_type = GPT_TIMER;
    int8_t tindex = FspTimer::get_available_timer(timer_type);
    if (tindex < 0) {
        tindex = FspTimer::get_available_timer(timer_type, true);
    }
    if (tindex < 0)
        return false;
    FspTimer::force_use_of_pwm_reserved_timer();
    if (!motorTimers.begin(TIMER_MODE_PERIODIC, timer_type, tindex, rate, 0.0f, motors_tick))
        return false;
    if (!motorTimers.setup_overflow_irq())
        return false;
    if (!motorTimers.open())
        return false;
    if (!motorTimers.start())
        return false;
    return true;
}

void motors_setup() {

    motors[0].side = LEFT;
    motors[0].period = IDLE_PERIOD;
    motors[0].nextTickTime = 0.f;
    motors[0].pulValue = 0;
    motors[0].pins.dir = 0;
    motors[0].pins.ena = 0;
    motors[0].pins.pul = 0;
    motors[0].ticksHistory = 0;
    motors[0].speed = 0.f;

    motors[1].side = RIGHT;
    motors[1].period = IDLE_PERIOD;
    motors[1].nextTickTime = 0.f;
    motors[1].pulValue = 0;
    motors[1].pins.dir = 0;
    motors[1].pins.ena = 0;
    motors[1].pins.pul = 0;
    motors[1].ticksHistory = 0;
    motors[1].speed = 0.f;

    for (int motorId = 0; motorId < NUM_MOTORS; motorId++) {
        motor_on(true, motorId);
        bool isClockwise;
        switch (motors[motorId].side) {
        case LEFT:
            isClockwise = false;
            break;
        case RIGHT:
            isClockwise = true;
            break;
        default:
            Serial.println("Undefined side");
        }
        move_clockwise(isClockwise, motorId);
        motors[motorId].rotationForward = 1;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////    INTERFACE    ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

inline float speed_to_period(const float speed) { return (2.f * M_PI * WHEELS_RADIUS) / (WHEELS_TICKS_PER_REVOLUTION * speed); }

inline float angular_speed_to_linear_speed(const float angularSpeed) { return -angularSpeed * WHEELS_SPACING * 0.5f; }

inline float ticks_to_linear_distance(const int ticks) { return 2 * WHEELS_RADIUS * M_PI * ((float)ticks) / WHEELS_TICKS_PER_REVOLUTION; }

void motor_set_speed(const float speed, const Side_t side) {
    const int motorId = side == LEFT ? 0 : 1;
    bool isClockwise;
    switch (motors[motorId].side) {
    case LEFT:
        isClockwise = false;
        break;
    case RIGHT:
        isClockwise = true;
        break;
    default:
        Serial.println("Undefined side");
    }

    if (speed < 0.f) {
        move_clockwise(!isClockwise, motorId);
        motors[motorId].period = 1e6f * speed_to_period(fabs(speed));
        motors[motorId].rotationForward = -1;
    } else if (speed == 0.f) {
        motor_on(false, motorId);
        move_clockwise(isClockwise, motorId);
        motors[motorId].period = IDLE_PERIOD;
        motors[motorId].rotationForward = 0;
    } else { // speed > 0.f
        move_clockwise(isClockwise, motorId);
        motors[motorId].period = 1e6f * speed_to_period(fabs(speed));
        motors[motorId].rotationForward = 1;
    }
    motors[motorId].nextTickTime = micros() + motors[motorId].period;
}

void pami_set_speed(const float linearSpeed, const float angularSpeed) {
    const float angularLinearizedSpeed = angular_speed_to_linear_speed(angularSpeed);
    const float leftSpeed = linearSpeed + angularLinearizedSpeed;
    const float rightSpeed = linearSpeed - angularLinearizedSpeed;

    if (motors[0].speed != leftSpeed && motors[1].speed != rightSpeed) {
        noInterrupts();
        motor_set_speed(leftSpeed, LEFT);
        motor_set_speed(rightSpeed, RIGHT);
        interrupts();
        motors[0].speed = leftSpeed;
        motors[1].speed = rightSpeed;
    }
}

void pami_get_displacement(float *x, float *y, float *angle) {
    noInterrupts();
    const int ticksLeft = motors[0].ticksHistory;
    const int ticksRight = motors[1].ticksHistory;
    motors[0].ticksHistory = 0;
    motors[1].ticksHistory = 0;
    interrupts();

    const float distLeft = ticks_to_linear_distance(ticksLeft);
    const float distRight = ticks_to_linear_distance(ticksRight);

    const float d = (distLeft + distRight) / 2;      // distance driven
    const float dtheta = (distRight - distLeft) / 2; // Angle made

    // Update
    *angle += dtheta / WHEELS_SPACING;
    *x += d * cosf(*angle);
    *y -= d * sinf(*angle);
}

//////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////       I/O       /////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

const int MAX_LENGTH = 20;
char incoming_Byte = '\r';
String text("");
bool is_end_of_line(char c) { return c == '\r' or c == '\n'; }

bool read_line() {
    if (is_end_of_line(incoming_Byte) or text.length() == MAX_LENGTH) {
        text = "";
        incoming_Byte = 'a';
    }
    while (Serial.available() && text.length() < MAX_LENGTH) {
        incoming_Byte = Serial.read();
        if (is_end_of_line(incoming_Byte)) {
            break;
        }
        text += incoming_Byte;
    }
    return is_end_of_line(incoming_Byte) or text.length() == MAX_LENGTH;
}

void *pathingHandler = nullptr;
void setup() {

    Serial.begin(19200);
    pathingHandler = pathing_get_handler(pami_get_displacement, pami_set_speed);
    motors_setup();
    if (!begin_timer(UPDATE_FREQ)) {
        Serial.println("Failed to init timer and interruption for filter.");
    }
}

unsigned long loopTime = 0;
void loop() {
    // read_line();
    // Serial.println(text);
    const unsigned long time = millis();
    if (loopTime < time) {

        pathing_update_speed(pathingHandler, PATHING_UPDATE_PERIOD / 1000.f);
        loopTime += PATHING_UPDATE_PERIOD;
    }
}

#ifndef ENABLE_ARDUINO

int main() {
    setup();
    while (1) {
        loop();
        usleep(1000);
    }
}

#endif // ENABLE_ARDUINO