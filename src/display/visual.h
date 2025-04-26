#ifndef _VISUAL_H_ 
#define _VISUAL_H_

struct Color_t {
    unsigned char r;
    unsigned char g;
    unsigned char b;
    unsigned char alpha;
};


void displayer_object_update_pos(int obj, float x, float y, double angle);
void displayer_update_target(int obj, float x, float y);
int displayer_add_object(double x, double y, double width, double height, double angle, struct Color_t c);
int displayer_add_target(double x, double y, double radius, struct Color_t c);
void register_goto_callback(void(*goto_callback)(double posX, double posY));
void displayer_start(unsigned int fps);
void displayer_stop();
int is_displayer_alive();


#endif // _VISUAL_H_
