#include "pathing.h"
#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#ifdef ENABLE_DISPLAY
    #include "visual.h"
#endif

typedef enum { MOTIONLESS = 0, ROTATION, LINEAR, CURVED } MotionState;
typedef enum { POSITION = 0, SPEED, WAITING } AsservMode;

typedef struct {
    float x;
    float y;
    float vLin;
    float accLin;
    float angle;
    float vAngle;
    float accAngle;
    float distToWPoint;
    float AngularDistToWPoint;
    bool wpChecked;
    float timeWaited;
    MotionState motionState;
    AsservMode asservMode;
} State;

typedef struct {
    unsigned int id;
    float x;
    float y;
    float vMax;
    float vaMax;
    float alpha;
    unsigned int stayStillTime;
    MotionState motionState;
} Waypoint;

typedef struct WaypointNode_t {
    struct WaypointNode_t *next;
    struct WaypointNode_t *previous;
    Waypoint wpoint;
} WaypointNode;

typedef struct {
    WaypointNode *start;
    WaypointNode *stop;
    unsigned int numNodes;
} Route;

struct PathHandler_t {
    Control_fcts controlFunctions;
    Route route;
    State state;
};

#define ACC_MAX 0.5f // m/(s^2)
#define ACC_ANGLE_MAX 1.6f
#define SPEED_LIN_MAX 0.1f // m/s
#define SPEED_DELTA_TRIGGER 0.005f
#define EPSILON 1e-4f

/*
--------------------------------------------------------------------
--------------------------    UTILS    -----------------------------
--------------------------------------------------------------------
*/

float distance(const float x1, const float y1, const float x2, const float y2) { return sqrtf((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)); }

float float_abs(const float x) { return x >= 0 ? x : -x; }

float float_max(const float x, const float y) { return x > y ? x : y; }

float float_min(const float x, const float y) { return x > y ? y : x; }
/*
--------------------------------------------------------------------
--------------------------    ROUTE    -----------------------------
--------------------------------------------------------------------
*/

/*
 * Distance ahead of slowing down :
 * Delta d > (v1 - v0)² / (2 * ACC_MAX)
 */

State State_create(const float x, const float y, const float vLin, const float accLin, const float angle, const float vAngle, const float accAngle,
                   const float distToWPoint, const float AngularDistToWPoint, const float timeWaited, MotionState motionState, AsservMode asservMode) {
    const State state = {.x = x,
                         .y = y,
                         .vLin = vLin,
                         .accLin = accLin,
                         .angle = angle,
                         .vAngle = vAngle,
                         .accAngle = accAngle,
                         .distToWPoint = distToWPoint,
                         .AngularDistToWPoint = AngularDistToWPoint,
                         .timeWaited = timeWaited,
                         .motionState = motionState,
                         .asservMode = asservMode,
                         .wpChecked = false};
    return state;
}

Waypoint Waypoint_create(const float x, const float y, const float vMax, const float vaMax, const float angle, const float stayStillTime, MotionState motionState) {
    Waypoint wpoint = {.id = UINT_MAX, .x = x, .y = y, .vMax = vMax, .vaMax = vaMax, .alpha = angle, .stayStillTime = stayStillTime , .motionState = motionState};
    return wpoint;
}

void Waypoint_destruct(Waypoint *wpoint) { wpoint->id = UINT_MAX; }

WaypointNode *WaypointNode_create(Waypoint wpoint) {
    WaypointNode *wnode = (WaypointNode *)malloc(sizeof(WaypointNode));
    wnode->next = NULL;
    wnode->previous = NULL;
    wnode->wpoint = wpoint;
    return wnode;
}

void WaypointNode_destruct(WaypointNode *wnode) {
    if (wnode->next != NULL && wnode->previous != NULL) {
        wnode->next->previous = wnode->previous;
        wnode->previous->next = wnode->next;
    } else if (wnode->next == NULL && wnode->previous != NULL) {
        wnode->previous->next = NULL;
    } else if (wnode->next != NULL && wnode->previous == NULL) {
        wnode->next->previous = NULL;
    }
    Waypoint_destruct(&wnode->wpoint);
    free(wnode);
}

Route Route_create() {
    Route route = {.start = NULL, .stop = NULL, .numNodes = 0};
    return route;
}

void Route_destruct(Route *route) {
    WaypointNode *ptr = route->start;
    WaypointNode *ptrNext = route->start->next;
    unsigned int numNodes = route->numNodes;
    while (ptrNext != NULL) {
        WaypointNode_destruct(ptr);
        route->numNodes--;
        ptr = ptrNext;
        ptrNext = ptrNext->next;
    }
    WaypointNode_destruct(ptr);
    route->numNodes--;
    if (route->numNodes != 0)
        printf("WARNING : %s -> Exepected to destroy %d nodes but destroyed %d "
               "nodes\n",
               __func__, numNodes, numNodes - route->numNodes);
}

void Route_add(Route *route, Waypoint wpoint) {
    WaypointNode *wnode = WaypointNode_create(wpoint);
    if ((route->start == NULL && route->stop != NULL)) {
        printf("WARNING : %s -> Route has end point but no starting point\n", __func__);
    }
    if (route->start != NULL && route->stop == NULL) {
        printf("WARNING : %s -> Route has a starting point but no end point\n", __func__);
    }

    if (route->start == NULL) {
        route->start = wnode;
        route->stop = wnode;
        wnode->previous = NULL;
        wnode->next = NULL;
    } else {
        wnode->previous = route->stop;
        wnode->next = NULL;
        route->stop->next = wnode;
        route->stop = wnode;
    }
    route->numNodes++;
}

void Route_remove(Route *route, unsigned int id) {
    if (id >= route->numNodes) {
        printf("WARNING : %s -> Expected id below %d but goot id %d\n", __func__, route->numNodes, id);
        return;
    }

    const int direction = (route->numNodes / 2) >= id ? 1 : -1;
    WaypointNode *ptr = direction == 1 ? route->start : route->stop;
    unsigned int elmtId = direction == 1 ? 0 : route->numNodes - 1;
    while (ptr != NULL && elmtId != id) {
        if (direction == 1)
            ptr = ptr->next;
        else
            ptr = ptr->previous;
        elmtId += direction;
    }
    if (id == 0) {
        route->start = route->start->next;
    }
    if (id == route->numNodes - 1) {
        route->stop = route->stop->previous;
    }
    WaypointNode_destruct(ptr);
    route->numNodes--;
}

bool is_waypoint_checked(State state, Waypoint wpoint, const float dt) {
    const float errorMargin = 0.02;            // state.vLin * dt;
    const float angleErrorMargin = M_PI / 180.f; // state.vAngle * dt;

    printf("Error angle : %f \t margin : %f\n", float_abs(state.angle - wpoint.alpha), angleErrorMargin);
    if (float_abs(state.angle - wpoint.alpha) > angleErrorMargin)
        return false;

    printf("Error dist : %f \t margin : %f\n", distance(wpoint.x, wpoint.y, state.x, state.y), errorMargin);
    if (distance(wpoint.x, wpoint.y, state.x, state.y) > errorMargin)
        return false;

    return true;
}

float route_get_next_linear_acc(State *state, Route *route, const float dt) {
    if (route->start->next == NULL)
        return 0.f;
    switch (state->motionState) {
    case MOTIONLESS:
        return float_abs(state->vLin) * (state->vLin >= 0 ? -1 : 1);

    case ROTATION:
        return (float_abs(state->vLin)) * (state->vLin >= 0 ? -1 : 1);

    case LINEAR: {
        const Waypoint *nextWPoint = &route->start->next->wpoint;
        // const float slowDownTrigger =  dt * float_abs(state->vLin - nextWPoint->vMax) * state->vLin / (ACC_MAX *2);
        // v(t) = ACC * t;
        // On cherche t tq v(t) = vLin
        // t = vLin / Acc
        // d(t) = ACC * t**2 / 2
        // const float slowDownTrigger =  ( float_abs(state->vLin - nextWPoint->vMax) / (ACC_MAX )) * (float_abs(state->vLin - nextWPoint->vMax) / (ACC_MAX )) * ACC_MAX / 2;
        const float dv = float_abs(nextWPoint->vMax - state->vLin);
        const float slowDownTrigger = (dv * dv / (2* ACC_MAX)) + ((nextWPoint->vMax * dv) / ACC_MAX);
        printf("Trigger : %f\n", slowDownTrigger);
        // Delai point haut et bas :
        if (state->distToWPoint <= slowDownTrigger) {
            return float_min(float_abs(nextWPoint->vMax - state->vLin)*dt, ACC_MAX*dt) * (nextWPoint->vMax - state->vLin >= 0 ? 1 : -1);
        }
        const Waypoint *currentWPoint = &route->start->wpoint;
        return float_min(float_abs((currentWPoint->vMax - state->vLin)*dt), ACC_MAX*dt) * (currentWPoint->vMax - state->vLin >= 0 ? 1 : -1);
    }

    case CURVED:
        printf("WARNING : %s -> CURVED motion state not implemented yet\n", __func__);
        return 0.f;

    default:
        printf("WARNING : %s -> Unknown motion state\n", __func__);
        return 0.f;
    }
}

float route_get_next_angular_acc(State *state, Route *route, const float dt) {
    if (route->start->next == NULL)
        return 0.f;

    switch (state->motionState) {
    case MOTIONLESS:
        return (float_abs(state->vAngle)) * (state->vAngle >= 0 ? 1 : -1);

    case ROTATION: {
        const Waypoint *nextWPoint = &route->start->next->wpoint;
        // const float slowDownTrigger = (state->vAngle - nextWPoint->vaMax) * (state->vAngle - nextWPoint->vaMax) / (2 * ACC_ANGLE_MAX);
        // const float slowDownTrigger = dt * float_abs(state->vAngle - nextWPoint->vaMax) * state->vAngle / (ACC_ANGLE_MAX * 2);
        const float dv = nextWPoint->vaMax - state->vAngle;
        const float slowDownTrigger = (dv * dv / (2* ACC_ANGLE_MAX)) + ((nextWPoint->vaMax * dv) / ACC_ANGLE_MAX);
        printf("Trigger : %f %f\n", slowDownTrigger, state->AngularDistToWPoint);
        if (float_abs(state->AngularDistToWPoint) <= slowDownTrigger) {
            return float_min(float_abs(nextWPoint->vaMax - state->vAngle) *dt, ACC_ANGLE_MAX * dt) * (nextWPoint->vaMax - state->vAngle >= 0 ? 1 : -1);
        }
        const Waypoint *currentWPoint = &route->start->wpoint;
        return float_min(float_abs(currentWPoint->vaMax - state->vAngle), ACC_ANGLE_MAX) *dt* (currentWPoint->vaMax - state->vAngle >= 0 ? 1 : -1);
    }

    case LINEAR:
        return (float_abs(state->vAngle)) * (state->vAngle >= 0 ? -1 : 1);

    case CURVED:
        printf("WARNING : %s -> CURVED motion state not implemented yet\n", __func__);
        return 0.f;

    default:
        printf("WARNING : %s -> Unknown motion state\n", __func__);
        return 0.f;
    }
}

PathHandler pathing_get_handler(const Control_fcts controlFunctions) {
    struct PathHandler_t *handler = (struct PathHandler_t *)malloc(sizeof(struct PathHandler_t));
    handler->controlFunctions = controlFunctions;

    Route route = Route_create();

    // const Waypoint wp1 = Waypoint_create(0.1, 1.2, 1.5, 0, 0, LINEAR);
    // const Waypoint wp2 = Waypoint_create(2.0, 1.2, 0, M_PI / (2), 0, ROTATION);
    // const Waypoint wp3 = Waypoint_create(2.0, 1.2, 0.4, 0, M_PI / 2, LINEAR);
    // const Waypoint wp4 = Waypoint_create(2.0, 0.2, 0, -M_PI, M_PI/2 , ROTATION);
    // const Waypoint wp5 = Waypoint_create(2.0, 0.2, 1., 0., -M_PI/2, LINEAR);
    // const Waypoint wp6 = Waypoint_create(2.0, 1.81, 0., 0., -M_PI/2, MOTIONLESS);


    const Waypoint wp1 = Waypoint_create(0.1, 1.2, 1.5, 0, 0,0., LINEAR);
    const Waypoint wp2 = Waypoint_create(2.1, 1.2, 0., 0., 0, 2., MOTIONLESS); 
    const Waypoint wp3 = Waypoint_create(2.1, 1.2, -1.5, 0., 0., 0.,LINEAR);
    const Waypoint wp4 = Waypoint_create(1.1, 1.2, 0., 0., 0, FLT_MAX, MOTIONLESS);
    const Waypoint wp5 = Waypoint_create(1.1, 1.2, 0, 0, 0, FLT_MAX, MOTIONLESS);

    Route_add(&route, wp1);
    Route_add(&route, wp2);
    Route_add(&route, wp3);
    Route_add(&route, wp4);
    Route_add(&route, wp5);
    // Route_add(&route, wp6);

    handler->route = route;

    handler->state = State_create(0.1, 1.2, 0., 0., 0, 0, 0, distance(1.9, 1.2, 0.1, 1.2), 0, 0., LINEAR, SPEED);

#ifdef ENABLE_DISPLAY
    // Visual

    unsigned int fps = 60;
    displayer_start(fps);

    struct Color_t color = {100, 10, 190, 255};
    double pos[2] = {0., 0.};
    double alpha = 0.;
    displayer_add_object(pos[0], pos[1], .10, .20, alpha, color);
#endif

    return (PathHandler)handler;
}

void pathing_destroy_handler(const PathHandler _handler) {
    struct PathHandler_t *handler = (struct PathHandler_t *)_handler;
    Route_destruct(&handler->route);

    free(handler);
}

void pathing_mode_speed(struct PathHandler_t *handler, const float dt) {
    if (is_waypoint_checked(handler->state, handler->route.start->next->wpoint, dt) || handler->state.asservMode == POSITION) {
        handler->state.wpChecked = false;
        printf("Waypoint checked ! Adjusting ...\n");
        handler->state.asservMode = POSITION;
        handler->controlFunctions.set_pos_target_func(handler->state.distToWPoint, handler->state.AngularDistToWPoint);
        printf("%f %f \n", handler->state.distToWPoint, handler->state.AngularDistToWPoint);

    } else {
        handler->state.accLin = route_get_next_linear_acc(&handler->state, &handler->route, dt);
        handler->state.accAngle = route_get_next_angular_acc(&handler->state, &handler->route, dt);
        handler->state.vLin += handler->state.accLin;
        handler->state.vAngle += handler->state.accAngle;
        handler->controlFunctions.set_speed_func(handler->state.vLin, handler->state.vAngle);
    }
}

void pathing_mode_position(struct PathHandler_t *handler, const float dt) {
    if (handler->controlFunctions.pami_is_position_target_reached()) {
        printf("Adjusted ! Going to next waypoint !\n");
        if (handler->state.timeWaited > dt) {
            handler->state.asservMode = WAITING;
            handler->state.timeWaited = handler->route.start->wpoint.stayStillTime - dt;
        } else 
            handler->state.asservMode = SPEED;
        handler->state.vLin = 0.f;
        handler->state.vAngle = 0.f;
        Route_remove(&handler->route, 0);
        if (handler->route.start != NULL)
            handler->state.motionState = handler->route.start->wpoint.motionState;
        else {
            handler->state.motionState = MOTIONLESS;
            handler->state.asservMode = WAITING;
            handler->state.timeWaited = FLT_MAX;
        }
    }
}

void pathing_mode_waiting(struct PathHandler_t *handler, const float dt) {
    if (handler->state.timeWaited <= dt) {
        handler->state.timeWaited = 0.f;
        handler->state.asservMode = SPEED;
    } else {
        handler->state.timeWaited -= dt;
    }
}

void update_handler(struct PathHandler_t *handler) {
        handler->controlFunctions.odometry_func(&handler->state.x, &handler->state.y, &handler->state.angle);

        handler->state.distToWPoint =
            distance(handler->state.x, handler->state.y, handler->route.start->next->wpoint.x, handler->route.start->next->wpoint.y);

        handler->state.AngularDistToWPoint = handler->route.start->next->wpoint.alpha - handler->state.angle;
}

void pathing_update_speed(const PathHandler _handler, const float dt) {
    struct PathHandler_t *handler = (struct PathHandler_t *)_handler;
    if (handler->state.asservMode == SPEED) {
        update_handler(handler);
        pathing_mode_speed(handler, dt);
    }  
    if (handler->state.asservMode == POSITION) {
        update_handler(handler);
        pathing_mode_position(handler, dt);
    }
    if (handler->state.asservMode == WAITING) {
        pathing_mode_waiting(handler, dt);            
    }


    // Display
#ifdef ENABLE_DISPLAY
    if (!is_displayer_alive()) {
        handler->controlFunctions.pami_shutdown();
        displayer_stop();
        pathing_destroy_handler(_handler);
        exit(EXIT_SUCCESS);
    }
    displayer_object_update_pos(0, handler->state.x, handler->state.y, handler->state.angle);
#endif
    printf("x : %f \t  y :%f \t a : %f \t v : %f \t va : %f\n", handler->state.x, handler->state.y, handler->state.angle, handler->state.vLin,
            handler->state.vAngle);
}
