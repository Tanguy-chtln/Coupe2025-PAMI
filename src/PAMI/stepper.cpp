#include "pami_settings.h"
#include "pathing.h"
#include <limits.h>
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
#define POSITION_MODE_SPEED 0.03f

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
    int ticksWorkLoad;
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
    motors[motorId].ticksWorkLoad--;
}

void motors_tick(timer_callback_args_t __attribute((unused)) * p_args) {
    const unsigned long time = micros();
    for (int motorId = 0; motorId < NUM_MOTORS; motorId++) {
        if (motors[motorId].nextTickTime <= time && motors[motorId].ticksWorkLoad) {
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

bool shutdown_timer() {
    // Stop the timer
    if (!motorTimers.stop()) {
        return false; // Return false if stopping fails
    }
    
    // Close the timer
    if (!motorTimers.close()) {
        return false; // Return false if closing fails
    }
    
    // Optionally, release any resources allocated for the timer
    FspTimer::release_pwm_reserved_timer();

    return true; // Return true to indicate a successful shutdown
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
    motors[0].ticksWorkLoad = 0;
    motors[0].speed = 0.f;

    motors[1].side = RIGHT;
    motors[1].period = IDLE_PERIOD;
    motors[1].nextTickTime = 0.f;
    motors[1].pulValue = 0;
    motors[1].pins.dir = 0;
    motors[1].pins.ena = 0;
    motors[1].pins.pul = 0;
    motors[1].ticksHistory = 0;
    motors[1].ticksWorkLoad = 0;
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

inline int linear_distance_to_ticks(const float linearDistance) {
    return (int)(linearDistance * WHEELS_TICKS_PER_REVOLUTION / (2 * WHEELS_RADIUS * M_PI));
}

inline int angular_delta_to_ticks(const float angleDelta) {
    return (int)ceil(angleDelta * WHEELS_SPACING * WHEELS_TICKS_PER_REVOLUTION / (2 * M_PI * WHEELS_RADIUS));
}

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
    motors[motorId].ticksWorkLoad = INT_MAX;
    // motors[motorId].nextTickTime = micros() + motors[motorId].period;
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

void pami_set_target_position(const float deltaPos, const float deltaAngle) {
    if (fabs(deltaPos) < fabs(deltaAngle)) {

        const int angularLinearizedTicks = (deltaAngle <= 1e-5f) && (deltaAngle >= -1e-5f) ? 0 : angular_delta_to_ticks(fabs(deltaAngle));

        const int leftTicksWorkLoad = angularLinearizedTicks;
        const int rightTicksWorkLoad = angularLinearizedTicks;
        const float leftSpeed = POSITION_MODE_SPEED * (deltaAngle >= 0 ? -1 : 1);
        const float rightSpeed = -leftSpeed;

        noInterrupts();
        motor_set_speed(leftSpeed, LEFT);
        motor_set_speed(rightSpeed, RIGHT);
        motors[0].ticksWorkLoad = leftTicksWorkLoad;
        motors[1].ticksWorkLoad = rightTicksWorkLoad;
        interrupts();
        printf("Called with deltaPos, deltaAngle : %f %f\nSetting target position at ticks : %d %d\t, speed : %f %f\n", deltaPos, deltaAngle,
               leftTicksWorkLoad, rightTicksWorkLoad, leftSpeed, rightSpeed);
        printf("------------------%d\n", angular_delta_to_ticks(fabs(deltaAngle)));
    } else {
        const int linearTicksWorkLoad = (deltaPos <= 1e-5f) && (deltaPos >= -1e-5f) ? 0 : linear_distance_to_ticks(fabs(deltaPos));

        const int leftTicksWorkLoad = linearTicksWorkLoad;
        const int rightTicksWorkLoad = linearTicksWorkLoad;
        const float leftSpeed = POSITION_MODE_SPEED * (deltaPos >= 0 ? 1 : -1);
        const float rightSpeed = leftSpeed;

        noInterrupts();
        motor_set_speed(leftSpeed, LEFT);
        motor_set_speed(rightSpeed, RIGHT);
        motors[0].ticksWorkLoad = leftTicksWorkLoad;
        motors[1].ticksWorkLoad = rightTicksWorkLoad;
        interrupts();
        printf("Called with deltaPos, deltaAngle : %f %f\nSetting target position at ticks : %d %d\t, speed : %f %f", deltaPos, deltaAngle,
               leftTicksWorkLoad, rightTicksWorkLoad, leftSpeed, rightSpeed);
    }
}

bool pami_is_position_target_reached() {
    printf("%d %d\n", motors[0].ticksWorkLoad, motors[1].ticksWorkLoad);
    if (motors[0].ticksWorkLoad != 0 || motors[1].ticksWorkLoad != 0)
        return false;
    return true;
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

unsigned long loopTime = 0;
void pami_shutdown() {
    shutdown_timer();
    loopTime = ULONG_MAX;
}

PathHandler pathingHandler = nullptr;
void setup() {

    Serial.begin(19200);
    Control_fcts controlFunctions = {.odometry_func = pami_get_displacement,
                                     .set_speed_func = pami_set_speed,
                                     .set_pos_target_func = pami_set_target_position,
                                     .pami_is_position_target_reached = pami_is_position_target_reached,
                                     .pami_shutdown = pami_shutdown};
    pathingHandler = pathing_get_handler(controlFunctions);
    motors_setup();

    if (!begin_timer(UPDATE_FREQ)) {
        Serial.println("Failed to init timer and interruption for filter.");
    }
}


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