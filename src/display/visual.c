#include <SDL2/SDL.h>
#include <SDL2/SDL_render.h>
#include <stdio.h>
#include <math.h>
#include <limits.h>
#include <stdlib.h>
#include <assert.h>
#include <pthread.h>
#include <unistd.h>
#include <inttypes.h>
#include <SDL2/SDL_image.h>
#include <stdbool.h>

#include "visual.h"

#define WINDOW_WIDTH 1500
#define WINDOW_HEIGHT 1000
#define BACKGROUND_WIDTH 3000
#define BACKGROUND_HEIGHT 2000
#define FPS_DISPLAY_FREQ 0.5

#ifndef RESSOURCES_DIR
    #define RESSOURCES_DIR "ressources"
#endif

#define BACKGROUND_IMG RESSOURCES_DIR "/playmat_2025_FINAL.png"


inline float MINf(float a, float b) {return a < b ? a : b;};

struct displayer_object_t {
    SDL_FRect rect;
    double alpha;
    SDL_Color color;
    SDL_Texture* rectTexture;
};


struct displayer_env_t {
    SDL_Window *window;
    SDL_Renderer *renderer;
    Uint32 lastDisplayTime;
    Uint32 fpsDisplayCountdown;
    int fpsAcc;
    int fpsAccNum;
    float windowWidth;
    float windowHeight;
    float rescaleFactor;

    struct displayer_object_t* objects;
    SDL_Texture *backgroundTexture;
    unsigned int num_objects;
    unsigned int max_num_objects;
    bool startup;
    bool isRunning;
    bool redraw;
    pthread_mutex_t drawMtx;
    pthread_t worker;
    unsigned int fpsRate;
};

struct displayer_env_t env;

float length_to_pixels(const float x) {
    return x *1000 ;
}

void sdl_error() {
    SDL_Log("ERROR > %s\n", SDL_GetError());
    exit(EXIT_FAILURE);
}



void displayer_init() {
    SDL_Window *window = NULL;
    SDL_Renderer *renderer = NULL;

    if (SDL_Init(SDL_INIT_VIDEO) != 0)  
        sdl_error();
    printf("SDL_VIDEO initialized\n");

    if (IMG_Init(IMG_INIT_PNG) == 0) {
        printf("Could not initialize SDL_image: %s\n", IMG_GetError());
        exit(EXIT_FAILURE);
    }

    window = SDL_CreateWindow("PAMI",
            SDL_WINDOWPOS_CENTERED,
            SDL_WINDOWPOS_CENTERED,
            WINDOW_WIDTH, WINDOW_HEIGHT, 
            SDL_WINDOW_SHOWN);

    if (window == NULL) 
        sdl_error();
    printf("Window created\n");

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (renderer == NULL) 
        sdl_error();

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);
    SDL_SetWindowResizable(window, (SDL_bool) true);

SDL_Surface* imageSurface = IMG_Load(BACKGROUND_IMG);
    if (!imageSurface) {
        printf("Could not load image: %s\n", IMG_GetError());
        exit(EXIT_FAILURE);
    }

    env.backgroundTexture = SDL_CreateTextureFromSurface(renderer, imageSurface);
    SDL_FreeSurface(imageSurface);
    if (!env.backgroundTexture) {
        fprintf(stderr, "Failed to load SVG as texture\n");
        exit(EXIT_FAILURE);
    }


    env.window = window;
    env.renderer = renderer;
    env.lastDisplayTime = SDL_GetTicks();
    env.fpsDisplayCountdown = 1000. / FPS_DISPLAY_FREQ;
    env.fpsAccNum = 0;
    env.fpsAcc = 0;
    env.max_num_objects = 0;
    env.windowWidth = WINDOW_WIDTH;
    env.windowHeight = WINDOW_HEIGHT;
    env.rescaleFactor = MINf(1.f * WINDOW_WIDTH / BACKGROUND_WIDTH, 1.f * WINDOW_HEIGHT / BACKGROUND_HEIGHT);
}

void displayer_quit() {
    for (unsigned int i = 0; i < env.num_objects; i++) {
        SDL_DestroyTexture(env.objects[i].rectTexture);
    }
    SDL_DestroyTexture(env.backgroundTexture);

    if (env.max_num_objects != 0 && env.objects != NULL)
        free(env.objects);

    SDL_DestroyRenderer(env.renderer);
    SDL_DestroyWindow(env.window);
    SDL_Quit();
}



SDL_Texture* create_rectangle_texture(SDL_Renderer* renderer, SDL_Color color, int width, int height) {

    SDL_Surface* surface = SDL_CreateRGBSurfaceWithFormat(0, width, height, 32, SDL_PIXELFORMAT_RGBA32);
    if (!surface) {
        SDL_Log("Failed to create surface: %s", SDL_GetError());
        return NULL; 
    }

    SDL_FillRect(surface, NULL, SDL_MapRGBA(surface->format, color.r, color.g, color.b, color.a));

    SDL_Color eyeColor = {255, 255, 255, 255}; 
    SDL_Color pupilColor = {0, 0, 0, 255};     
    int eyeRadius = 10;                        
    int pupilRadius = 5;                       

    // Left eye
    int leftEyeX = width / 4; 
    int leftEyeY = height / 4; 
    for (int w = -eyeRadius; w <= eyeRadius; w++) {
        for (int h = -eyeRadius; h <= eyeRadius; h++) {
            if (w * w + h * h <= eyeRadius * eyeRadius) {
                // Eye
                int px = leftEyeX + w;
                int py = leftEyeY + h;
                if (px >= 0 && py >= 0 && px < width && py < height) {
                    ((Uint32*)surface->pixels)[py * surface->w + px] = SDL_MapRGBA(surface->format, eyeColor.r, eyeColor.g, eyeColor.b, eyeColor.a);
                }
                // Pupil
                if (w * w + h * h <= pupilRadius * pupilRadius) {
                    ((Uint32*)surface->pixels)[py * surface->w + px] = SDL_MapRGBA(surface->format, pupilColor.r, pupilColor.g, pupilColor.b, pupilColor.a);
                }
            }
        }
    }

    // Right eye
    int rightEyeX = 3 * width / 4;
    for (int w = -eyeRadius; w <= eyeRadius; w++) {
        for (int h = -eyeRadius; h <= eyeRadius; h++) {
            if (w * w + h * h <= eyeRadius * eyeRadius) {
                // Eye
                int px = rightEyeX + w;
                int py = leftEyeY + h;
                if (px >= 0 && py >= 0 && px < width && py < height) {
                    ((Uint32*)surface->pixels)[py * surface->w + px] = SDL_MapRGBA(surface->format, eyeColor.r, eyeColor.g, eyeColor.b, eyeColor.a);
                }
                // Pupil
                if (w * w + h * h <= pupilRadius * pupilRadius) {
                    ((Uint32*)surface->pixels)[py * surface->w + px] = SDL_MapRGBA(surface->format, pupilColor.r, pupilColor.g, pupilColor.b, pupilColor.a);
                }
            }
        }
    }

    // Create a texture from the surface
    SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
    SDL_FreeSurface(surface);
    if (!texture) { 
        SDL_Log("Failed to create texture : %s", SDL_GetError());
        return NULL;
    }
    return texture;
}



void draw_rotated_rect(SDL_Renderer *renderer, struct displayer_object_t* object) {
    // Set the destination rectangle
    SDL_Rect destRect = {
        (int) object->rect.x * env.rescaleFactor ,
        (int) object->rect.y * env.rescaleFactor ,
        (int) object->rect.w * env.rescaleFactor,
        (int) object->rect.h * env.rescaleFactor
    };
    // Render the texture with rotation
    SDL_RenderCopyEx(renderer, object->rectTexture, NULL, &destRect, (-object->alpha + M_PI/2) * 180.f / M_PI, NULL, SDL_FLIP_NONE);
}

int displayer_add_object(double _x, double _y, double _width, double _height, double angle, struct Color_t c) {
    const double x = length_to_pixels(_x);
    const double y = length_to_pixels(_y);
    const double width = length_to_pixels(_width) ;
    const double height = length_to_pixels(_height) ;
    SDL_Color sdlColor = {(Uint8) c.r, (Uint8) c.g, (Uint8) c.b, (Uint8) c.alpha};
    struct displayer_object_t object = {{x - width / 2, y - height / 2, width, height}, angle, sdlColor, NULL};
    pthread_mutex_lock(&env.drawMtx);
    const int objectId = env.num_objects;

    if (env.max_num_objects == 0) {
        const unsigned int startup_obj_num = 10;
        env.objects =  (struct displayer_object_t *) calloc(startup_obj_num, sizeof(struct displayer_object_t));
        env.max_num_objects = startup_obj_num;
    } else if (env.max_num_objects <= env.num_objects){
        printf("Non sens\n");
        const int widening_multiplier = 2;
        env.objects = (struct displayer_object_t *) realloc(env.objects, env.max_num_objects * widening_multiplier * sizeof(struct displayer_object_t));
        if (env.objects == NULL) {
            fprintf(stderr, "Memory allocation failed\n");
            exit(EXIT_FAILURE);
        }

        env.max_num_objects *= 2;
    }

    env.objects[objectId] = object;
    env.num_objects++;
    env.redraw = true;
    pthread_mutex_unlock(&env.drawMtx);
    return objectId;
}

void displayer_object_update_pos(int obj, float _x, float _y, double angle) {
    assert(obj >= 0 && (unsigned int) obj < env.num_objects);
    struct displayer_object_t *object = env.objects + obj;

    const float x = length_to_pixels(_x);
    const float y = length_to_pixels(_y);

    pthread_mutex_lock(&env.drawMtx);
    object->rect.x = x - object->rect.w / 2;
    object->rect.y = y - object->rect.h / 2;
    object->alpha = angle;
    env.redraw = true;
    pthread_mutex_unlock(&env.drawMtx);
}


void displayer_display() {
    if (env.redraw) {
        SDL_Rect backgroundRect = {0, 0, BACKGROUND_WIDTH * env.rescaleFactor, BACKGROUND_HEIGHT * env.rescaleFactor}; 
        SDL_SetRenderDrawColor(env.renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
        SDL_RenderClear(env.renderer);
        SDL_RenderCopy(env.renderer, env.backgroundTexture, NULL, &backgroundRect);
        SDL_SetRenderDrawColor(env.renderer, 255, 0, 0, SDL_ALPHA_OPAQUE);
        // pthread_mutex_lock(&env.drawMtx);
        for (unsigned int objectId = 0; objectId < env.num_objects; objectId++) {
            struct displayer_object_t *object = env.objects + objectId;
            draw_rotated_rect(env.renderer, object);
        }
        env.redraw = false;
        // pthread_mutex_unlock(&env.drawMtx);

        SDL_RenderPresent(env.renderer);
    }

    const Uint32 ellapsedTime = SDL_GetTicks() - env.lastDisplayTime; // ms
    env.fpsAcc += ellapsedTime ? 1000 / ellapsedTime : INT_MAX;
    env.fpsAccNum++;
    env.lastDisplayTime = SDL_GetTicks();

    if (env.fpsDisplayCountdown > ellapsedTime) 
        env.fpsDisplayCountdown -= ellapsedTime;
    else {
        char title[128];
        sprintf(title, "PAMI | FPS : %d", env.fpsAcc / env.fpsAccNum);
        SDL_SetWindowTitle(env.window,title);
        env.fpsDisplayCountdown = 1000. / FPS_DISPLAY_FREQ;
        env.fpsAcc = 0;
        env.fpsAccNum = 0;
    }
}


void *worker(void *) {
    displayer_init();
    Uint32 startTime, endTime, ellapsedTime;

    SDL_Event event;
    env.startup = true;
    env.isRunning = true;
    unsigned int initializedObjects = 0;

    while (env.isRunning) {
        startTime = SDL_GetTicks();
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT)
                env.isRunning = false;
            if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.sym == SDLK_ESCAPE)  
                    env.isRunning = false;
            }
            if (event.type == SDL_WINDOWEVENT) {
                if (event.window.event == SDL_WINDOWEVENT_RESIZED) {
                    env.windowWidth = event.window.data1; // New width
                    env.windowHeight = event.window.data2; // New height
                    env.rescaleFactor = MINf(env.windowWidth / BACKGROUND_WIDTH, env.windowHeight / BACKGROUND_HEIGHT);
                }
            }

        }
        if (initializedObjects < env.num_objects) {
            pthread_mutex_lock(&env.drawMtx);
            for (unsigned int objectId = initializedObjects; objectId < env.num_objects; objectId++) {
                struct displayer_object_t *object = env.objects + objectId;
                object->rectTexture = create_rectangle_texture(env.renderer, object->color, object->rect.w , object->rect.h);
                initializedObjects++;
            }
            pthread_mutex_unlock(&env.drawMtx);
        }
        displayer_display();

        endTime = SDL_GetTicks();
        ellapsedTime = endTime - startTime;
        if (ellapsedTime < 1000.f / env.fpsRate)
            SDL_Delay(1000.f / env.fpsRate - ellapsedTime);
    }

    displayer_quit();
    return NULL;
}


void displayer_start(unsigned int fps) {
    env.startup = false;
    env.fpsRate = fps;
    pthread_mutex_init(&env.drawMtx, NULL);
    pthread_create(&env.worker, NULL, worker, NULL);

    while(!env.startup)
        usleep(1000 / 32);
    usleep(100000);
}

void displayer_stop() {
    env.isRunning = false;
    pthread_join(env.worker, NULL);
    pthread_mutex_destroy(&env.drawMtx);
}

int is_displayer_alive() {
    return env.isRunning;
}

#ifdef ENABLE_DISPLAY_DEBUG

#define VISUAL_DEBUG_NUM_OBJECTS 2

int main() {
    unsigned int fps = 60;
    displayer_start(fps);

    struct Color_t colors[VISUAL_DEBUG_NUM_OBJECTS] = {{100,10, 190, 255}, {20, 150, 120, 255}};
    double pos[VISUAL_DEBUG_NUM_OBJECTS][2] = {{1.5, 0.5}, {1.5, 1.5}};
    double alpha[VISUAL_DEBUG_NUM_OBJECTS] = {0, -M_PI};
    double v[VISUAL_DEBUG_NUM_OBJECTS] = {0.000015, 0.000015};
    int objs[VISUAL_DEBUG_NUM_OBJECTS];
    for (int objectId = 0; objectId < VISUAL_DEBUG_NUM_OBJECTS; objectId++) { 
        objs[objectId] = displayer_add_object(pos[objectId][0], pos[objectId][1], 0.520, 0.190, alpha[objectId], colors[objectId]);
    }

    while (is_displayer_alive()) {
        for (int objectId = 0; objectId < VISUAL_DEBUG_NUM_OBJECTS; objectId++) {
            alpha[objectId] = alpha[objectId] - .0018 * M_PI / 180.;
            pos[objectId][0] += v[objectId] * cos(alpha[objectId]);
            pos[objectId][1] -= v[objectId] * sin(alpha[objectId]);
            displayer_object_update_pos(objs[objectId], pos[objectId][0], pos[objectId][1], alpha[objectId]);
        }
        usleep(1000 / fps);
    }

    displayer_stop();
}

#endif // VISUAL_DEBUG
