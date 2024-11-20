#ifndef _PATHING_H_
#define _PATHING_H_
#ifdef __cplusplus
extern "C" {
#endif
#include <stdbool.h>

typedef struct PathHandler_t *PathHandler;

typedef struct {
    void (*odometry_func)(float *, float *, float *);
    void (*set_speed_func)(const float, const float);
    void (*set_pos_target_func)(const float, const float);
    bool (*pami_is_position_target_reached)();
} Control_fcts;

PathHandler pathing_get_handler(const Control_fcts ctrl_fcts);

void pathing_destroy_handler(const PathHandler handler);

void pathing_update_speed(const PathHandler handler, const float dt);

#ifdef __cplusplus
}
#endif
#endif // _PATHING_H_