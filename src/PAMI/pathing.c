#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

typedef enum { MOTIONLESS = 0, ROTATION, LINEAR, CURVED } MotionState;
typedef enum { POSITION = 0, SPEED } AsservMode;

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

#define ACC_MAX 0.03f // m/(s^2)
#define ACC_ANGLE_MAX 1.f
#define SPEED_LIN_MAX 0.1f // m/s
#define SPEED_DELTA_TRIGGER 0.005f
#define EPSILON 1e-4f

/*
--------------------------------------------------------------------
--------------------------    UTILS    -----------------------------
--------------------------------------------------------------------
*/

inline float distance(const float x1, const float y1, const float x2, const float y2) { return sqrtf((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)); }

inline float float_abs(const float x) { return x >= 0 ? x : -x; }

inline float float_max(const float x, const float y) { return x > y ? x : y; }

inline float float_min(const float x, const float y) { return x > y ? y : x; }
/*
--------------------------------------------------------------------
--------------------------    ROUTE    -----------------------------
--------------------------------------------------------------------
*/

/*
 * Distance ahead of slowing down :
 * Delta d > (v1 - v0)² / (2 * ACC_MAX)
 */

Waypoint Waypoint_create(const float x, const float y, const float vMax, const float vaMax, const float angle, MotionState motionState) {
    Waypoint wpoint = {.id = UINT_MAX, .x = x, .y = y, .vMax = vMax, .vaMax = vaMax, .alpha = angle, .motionState = motionState};
    return wpoint;
}

void Waypoint_destruct(Waypoint *wpoint) { wpoint->id = UINT_MAX; }

WaypointNode *WaypointNode_create(Waypoint wpoint) {
    WaypointNode *wnode = malloc(sizeof(WaypointNode));
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
    while (ptr != NULL) {
        WaypointNode_destruct(ptr);
        route->numNodes--;
        ptr = ptrNext;
        ptrNext = ptrNext->next;
    }
    if (route->numNodes != 0)
        printf("WARNING : %s -> Exepected to destroy %d nodes but destroyed %d "
               "nodes\n",
               __func__, numNodes, numNodes - route->numNodes);
}

void Route_add(Route *route, Waypoint wpoint) {
    WaypointNode *wnode = WaypointNode_create(wpoint);
    if ((route->start == NULL && route->stop != NULL) || (route->start != NULL && route->stop == NULL)) {
        printf("WARNING : %s -> Route has end point but no starting point\n", __func__);
    }
    if (route->start == NULL) {
        route->start = wnode;
        route->start = wnode;
        wnode->previous = NULL;
        wnode->next = NULL;
    } else {
        wnode->previous = route->stop;
        wnode->next = NULL;
        route->stop->next = wnode;
    }
    route->numNodes++;
}

void Route_remove(Route *route, unsigned int id) {
    if (id >= route->numNodes) {
        printf("WARNING : %s -> Expected id below %d but goot id %d\n", __func__, route->numNodes, id);
        return;
    }

    const int direction = (route->numNodes / 2) < id ? 1 : -1;
    WaypointNode *ptr = direction == 1 ? route->start : route->stop;
    unsigned int elmtId = direction == 1 ? 0 : route->numNodes - 1;
    while (ptr != NULL && elmtId != id) {
        if (direction == 1)
            ptr = ptr->next;
        else
            ptr = ptr->previous;
        elmtId += direction;
    }

    WaypointNode_destruct(ptr);
}

bool is_waypoint_checked(State state, Waypoint wpoint, const float dt) {
    const float errorMargin = state.vLin * dt;
    const float angleErrorMargin = state.vAngle * dt;

    if (float_abs(state.angle - wpoint.alpha) > angleErrorMargin)
        return false;

    if (distance(wpoint.x, wpoint.y, state.x, state.y) > errorMargin)
        return false;

    return true;
}

float route_get_next_linear_acc(State *state, Route *route) {
    if (route->start->next == NULL)
        return 0.f;

    switch (state->motionState) {
    case MOTIONLESS:
        if (float_abs(state->vLin) > EPSILON)
            return float_min(float_abs(state->vLin), ACC_MAX) * (state->vLin >= 0 ? 1 : -1);
        return 0.f;

    case ROTATION:
        if (float_abs(state->vLin) > EPSILON)
            return float_min(float_abs(state->vLin), ACC_MAX) * (state->vLin >= 0 ? 1 : -1);
        return 0.f;

    case LINEAR: {
        const Waypoint *nextWPoint = &route->start->next->wpoint;
        const float slowDownTrigger = (state->vLin - nextWPoint->vMax) * (state->vLin - nextWPoint->vMax) / (2 * ACC_MAX);
        if (state->distToWPoint <= slowDownTrigger) {
            return float_min(float_abs(nextWPoint->vMax - state->vLin), ACC_MAX) * (nextWPoint->vMax - state->vLin >= 0 ? 1 : -1);
        }
        const Waypoint *currentWPoint = &route->start->wpoint;
        return float_min(float_abs(currentWPoint->vMax - state->vLin), ACC_MAX) * (currentWPoint->vMax - state->vLin >= 0 ? 1 : -1);
    }

    case CURVED:
        printf("WARNING : %s -> CURVED motion state not implemented yet\n", __func__);
        return 0.f;

    default:
        printf("WARNING : %s -> Unknown motion state\n", __func__);
        return 0.f;
    }
}

float route_get_next_angular_acc(State *state, Route *route) {
    if (route->start->next == NULL)
        return 0.f;

    switch (state->motionState) {
    case MOTIONLESS:
        if (float_abs(state->vAngle) > EPSILON)
            return float_min(float_abs(state->vAngle), ACC_ANGLE_MAX) * (state->vAngle >= 0 ? 1 : -1);
        return 0.f;

    case ROTATION: {
        const Waypoint *nextWPoint = &route->start->next->wpoint;
        const float slowDownTrigger = (state->vAngle - nextWPoint->vaMax) * (state->vAngle - nextWPoint->vaMax) / (2 * ACC_ANGLE_MAX);
        if (state->distToWPoint <= slowDownTrigger) {
            return float_min(float_abs(nextWPoint->vaMax - state->vAngle), ACC_ANGLE_MAX) * (nextWPoint->vaMax - state->vAngle >= 0 ? 1 : -1);
        }
        const Waypoint *currentWPoint = &route->start->wpoint;
        return float_min(float_abs(currentWPoint->vaMax - state->vAngle), ACC_ANGLE_MAX) * (currentWPoint->vaMax - state->vAngle >= 0 ? 1 : -1);
    }

    case LINEAR:
        if (float_abs(state->vAngle) > EPSILON)
            return float_min(float_abs(state->vAngle), ACC_ANGLE_MAX) * (state->vAngle >= 0 ? 1 : -1);
        return 0.f;

    case CURVED:
        printf("WARNING : %s -> CURVED motion state not implemented yet\n", __func__);
        return 0.f;

    default:
        printf("WARNING : %s -> Unknown motion state\n", __func__);
        return 0.f;
    }
}
