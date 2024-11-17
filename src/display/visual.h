#ifndef _VISUAL_H_ 
#define _VISUAL_H_

struct Color_t {
    unsigned char r;
    unsigned char g;
    unsigned char b;
    unsigned char alpha;
};


void displayer_object_update_pos(int obj, float x, float y, double angle);
int displayer_add_object(double x, double y, double width, double height, double angle, struct Color_t c);
void displayer_start(unsigned int fps);
void displayer_stop();
int is_displayer_alive();

#endif // _VISUAL_H_
