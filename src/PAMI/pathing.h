#ifndef _PATHING_H_
#define _PATHING_H_
#ifdef __cplusplus
extern "C" {
#endif

void some_c_function();

void *pathing_get_handler(void (*odometry_func)(float *, float *, float *), void (*set_speed_func)(const float, const float));

void pathing_destroy_handler(void *handler);

void pathing_update_speed(void *handler, float dt);

#ifdef __cplusplus
}
#endif
#endif // _PATHING_H_