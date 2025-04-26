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

#define WINDOW_WIDTH 1500 // pixels 
#define WINDOW_HEIGHT 1000 // pixels 
#define BACKGROUND_WIDTH 3000 // mm 
#define BACKGROUND_HEIGHT 2000 // mm 
#define FPS_DISPLAY_FREQ 0.5 // Frequency at which frame rate display is updated (screen title)

#ifndef RESSOURCES_DIR
    #define RESSOURCES_DIR "ressources"
#endif

#define BACKGROUND_IMG RESSOURCES_DIR "/playmat_2025_FINAL.png" // Background image 


static inline float MINf(float a, float b) {return a < b ? a : b;};

typedef enum {
  ROBOT,
  TARGET
} obj_type_t;

// Struct for objects textures informations
struct displayer_object_t {
    SDL_FRect rect;
    double alpha;
    SDL_Color color;
    SDL_Texture* rectTexture;
    obj_type_t objType;
};

// Renderer informations 
static struct displayer_env_t {
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

    void (*goto_callback)(double posX, double posY);
} env;

// Converts meters to pixels 
static float length_to_pixels(const float x) {
    return x *1000 ;
}

static void sdl_error() {
    SDL_Log("ERROR > %s\n", SDL_GetError());
    exit(EXIT_FAILURE);
}


// Initialize displayer 
static void displayer_init() {
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
    env.goto_callback = NULL;
    env.redraw = true;
}

// Destroy displayer 
static void displayer_quit() {
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


// Create a robot-shaped texture 
static SDL_Texture* create_rectangle_texture(SDL_Renderer* renderer, SDL_Color color, int width, int height) {

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

// Creates a circle texture
SDL_Texture* create_circle_texture(SDL_Renderer* renderer, int radius, SDL_Color color) {
    int diameter = radius * 2;

    SDL_Surface* surface = SDL_CreateRGBSurfaceWithFormat(0, diameter, diameter, 32, SDL_PIXELFORMAT_RGBA32);
    if (!surface) {
        SDL_Log("Failed to create surface: %s", SDL_GetError());
        return NULL;
    }

    SDL_FillRect(surface, NULL, SDL_MapRGBA(surface->format, 0, 0, 0, 0));
    SDL_LockSurface(surface);
    Uint32* pixels = (Uint32*)surface->pixels;
    Uint32 pixelColor = SDL_MapRGBA(surface->format, color.r, color.g, color.b, color.a);

    for (int y = 0; y < diameter; ++y) {
        for (int x = 0; x < diameter; ++x) {
            int dx = x - radius;
            int dy = y - radius;
            if (dx*dx + dy*dy <= radius*radius) {
                pixels[y * surface->w + x] = pixelColor;
            }
        }
    }

    SDL_UnlockSurface(surface);
    SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
    SDL_FreeSurface(surface);

    if (!texture) {
        SDL_Log("Failed to create texture: %s", SDL_GetError());
    }

    return texture;
}

// Draw a rotated texture 
static void draw_rotated_rect(SDL_Renderer *renderer, struct displayer_object_t* object) {
    // Set the destination rectangle
    SDL_Rect destRect = {
        (int) (object->rect.x * env.rescaleFactor) ,
        (int) (object->rect.y * env.rescaleFactor) ,
        (int) (object->rect.w * env.rescaleFactor),
        (int) (object->rect.h * env.rescaleFactor)
    };
    // Render the texture with rotation
    SDL_RenderCopyEx(renderer, object->rectTexture, NULL, &destRect, (-object->alpha + M_PI/2) * 180.f / M_PI, NULL, SDL_FLIP_NONE);
}

static int displayer_add_object_all_type(struct displayer_object_t object) {
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

// Adds  new object to be displayed 
int displayer_add_object(double _x, double _y, double _width, double _height, double angle, struct Color_t c) {
    const double x = length_to_pixels(_x);
    const double y = length_to_pixels(_y);
    const double width = length_to_pixels(_width) ;
    const double height = length_to_pixels(_height) ;
    SDL_Color sdlColor = {(Uint8) c.r, (Uint8) c.g, (Uint8) c.b, (Uint8) c.alpha};
    struct displayer_object_t object = {{(float)(x - width / 2), (float)(y - height / 2), (float)width, (float)height}, angle, sdlColor, NULL, ROBOT};
    return displayer_add_object_all_type(object);
}

static void displayer_draw_target(SDL_Renderer *renderer, struct displayer_object_t *targetObj) {
    SDL_Texture *circleTexture = targetObj->rectTexture;
   if (!circleTexture) return;
    SDL_Rect destRect = {
        (int) (targetObj->rect.x * env.rescaleFactor) ,
        (int) (targetObj->rect.y * env.rescaleFactor) ,
        (int) (targetObj->rect.w * env.rescaleFactor),
        (int) (targetObj->rect.h * env.rescaleFactor)
    };

    SDL_RenderCopy(renderer, circleTexture, NULL, &destRect);
}

int displayer_add_target(double _x, double _y, double _radius, struct Color_t c) {
    const double x = length_to_pixels(_x);
    const double y = length_to_pixels(_y);
    const double r = length_to_pixels(_radius);
    SDL_Color sdlColor = {(Uint8) c.r, (Uint8) c.g, (Uint8) c.b, (Uint8) c.alpha};
    struct displayer_object_t object = {{(float)(x - r / 2), (float)(y - r / 2), (float)r*2, (float)r*2}, 0, sdlColor, NULL, TARGET};
    return displayer_add_object_all_type(object);
}

// Update the position of a declared object
void displayer_object_update_pos(int obj, float _x, float _y, double angle) {
    assert(obj >= 0 && (unsigned int) obj < env.num_objects);
    struct displayer_object_t *object = env.objects + obj;
    assert(object->objType == ROBOT);

    const float x = length_to_pixels(_x);
    const float y = length_to_pixels(_y);

    pthread_mutex_lock(&env.drawMtx);
    object->rect.x = x - object->rect.w / 2;
    object->rect.y = y - object->rect.h / 2;
    object->alpha = angle;
    env.redraw = true;
    pthread_mutex_unlock(&env.drawMtx);
}

void displayer_update_target(int obj, float _x, float _y) {
    assert(obj >= 0 && (unsigned int) obj < env.num_objects);
    struct displayer_object_t *object = env.objects + obj;
    assert(object->objType == TARGET);

    const float x = length_to_pixels(_x);
    const float y = length_to_pixels(_y);

    pthread_mutex_lock(&env.drawMtx);
    object->rect.x = x - object->rect.w / 2;
    object->rect.y = y - object->rect.h / 2;
    env.redraw = true;
    pthread_mutex_unlock(&env.drawMtx);
}

// Update screen
static void displayer_display() {
    if (env.redraw) {
        SDL_Rect backgroundRect = {0, 0, (int)(BACKGROUND_WIDTH * env.rescaleFactor), (int)(BACKGROUND_HEIGHT * env.rescaleFactor)}; 
        SDL_SetRenderDrawColor(env.renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
        SDL_RenderClear(env.renderer);
        SDL_RenderCopy(env.renderer, env.backgroundTexture, NULL, &backgroundRect);
        SDL_SetRenderDrawColor(env.renderer, 255, 0, 0, SDL_ALPHA_OPAQUE);
        // pthread_mutex_lock(&env.drawMtx);
        for (unsigned int objectId = 0; objectId < env.num_objects; objectId++) {
            struct displayer_object_t *object = env.objects + objectId;
            if (object->objType == ROBOT)
              draw_rotated_rect(env.renderer, object);
            else if (object->objType == TARGET)
              displayer_draw_target(env.renderer, object);
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

// Start display thread
static void *worker(void *) {
    displayer_init();
    Uint32 startTime, endTime, ellapsedTime;

    SDL_Event event;
    env.startup = true;
    env.isRunning = true;
    unsigned int initializedObjects = 0;

    while (env.isRunning) {
        startTime = SDL_GetTicks();
        while (SDL_PollEvent(&event)) { // User actions
            if (event.type == SDL_QUIT) 
                env.isRunning = false;
            if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.sym == SDLK_ESCAPE)  // Escaoe us pressed
                    env.isRunning = false;
            }
            if (event.type == SDL_WINDOWEVENT) {
                if (event.window.event == SDL_WINDOWEVENT_RESIZED) {
                    env.windowWidth = event.window.data1; // New width
                    env.windowHeight = event.window.data2; // New height
                    env.rescaleFactor = MINf(env.windowWidth / BACKGROUND_WIDTH, env.windowHeight / BACKGROUND_HEIGHT);
                }
            }
            if (event.type == SDL_MOUSEBUTTONDOWN && event.button.button == SDL_BUTTON_LEFT) {
                const int x = event.button.x;
                const int y = event.button.y;
                const double posX = x / (env.rescaleFactor * 1000);
                const double posY = y / (env.rescaleFactor*1000);
                if (env.goto_callback)
                  env.goto_callback(posX, posY);
            }

        }
        if (initializedObjects < env.num_objects) {
            pthread_mutex_lock(&env.drawMtx);
            for (unsigned int objectId = initializedObjects; objectId < env.num_objects; objectId++) {
                struct displayer_object_t *object = env.objects + objectId;
                if (object->objType == ROBOT)
                  object->rectTexture = create_rectangle_texture(env.renderer, object->color, object->rect.w , object->rect.h);
                else if (object->objType == TARGET)
                  object->rectTexture = create_circle_texture(env.renderer, object->rect.w/2, object->color);
                initializedObjects++;
            }
            pthread_mutex_unlock(&env.drawMtx);
        }
        displayer_display(); // Update screen

        endTime = SDL_GetTicks();
        ellapsedTime = endTime - startTime;
        if (ellapsedTime < 1000.f / env.fpsRate)
            SDL_Delay(1000.f / env.fpsRate - ellapsedTime);
    }

    displayer_quit();
    return NULL;
}

void register_goto_callback(void (*callback)(double posX, double posY)) {
    env.goto_callback = callback;
}

// Startup the displayer
void displayer_start(unsigned int fps) {
    env.startup = false;
    env.fpsRate = fps;
    pthread_mutex_init(&env.drawMtx, NULL);
    pthread_create(&env.worker, NULL, worker, NULL);

    while(!env.startup)
        usleep(1000 / 32);
    usleep(100000);
}

// Stops the displayer
void displayer_stop() {
    env.isRunning = false;
    pthread_join(env.worker, NULL);
    pthread_mutex_destroy(&env.drawMtx);
}

// Returns weither the displayer has been stopped 
int is_displayer_alive() {
    return env.isRunning;
}

#ifdef ENABLE_DISPLAY_DEBUG

#define VISUAL_DEBUG_NUM_OBJECTS 2

void debug_goto_callback(double x, double y) {
  printf("%lf %lf\n", x, y);
}

int main() {
    unsigned int fps = 60;
    displayer_start(fps);
    register_goto_callback(&debug_goto_callback);

    struct Color_t colors[VISUAL_DEBUG_NUM_OBJECTS] = {{100,10, 190, 255}, {20, 150, 120, 255}}; // Colors of each objects (R,G,B)
    double pos[VISUAL_DEBUG_NUM_OBJECTS][2] = {{1.5, 0.5}, {1.5, 1.5}}; // Position of each objects, in meters, from the upper left corner
    double alpha[VISUAL_DEBUG_NUM_OBJECTS] = {0, -M_PI}; // Angle of each object in radians
    double v[VISUAL_DEBUG_NUM_OBJECTS] = {0.000015, 0.000015}; // Speed of each object in m/s
    int objs[VISUAL_DEBUG_NUM_OBJECTS]; // Objects containers
    for (int objectId = 0; objectId < VISUAL_DEBUG_NUM_OBJECTS; objectId++) { 
        objs[objectId] = displayer_add_object(pos[objectId][0], pos[objectId][1], 0.520, 0.190, alpha[objectId], colors[objectId]); // Declare objects 
    // 0.52 and 0.19 are the objects dimensions in meters
    }

    while (is_displayer_alive()) { // While the user has not killed the window, continue to update the objects position 
        for (int objectId = 0; objectId < VISUAL_DEBUG_NUM_OBJECTS; objectId++) { 
            alpha[objectId] = alpha[objectId] - .0018 * M_PI / 180.; // Calculate new angle 
            pos[objectId][0] += v[objectId] * cos(alpha[objectId]); // Calculate new X position 
            pos[objectId][1] -= v[objectId] * sin(alpha[objectId]); // Calclulate new Y position 
            displayer_object_update_pos(objs[objectId], pos[objectId][0], pos[objectId][1], alpha[objectId]); // Update objects' position stored in the displayer 
        }
        usleep(1000 / fps); // Wait a little (different from the FPS of the displayer itself)
    }

    displayer_stop(); // Kill the displayer
}

#endif // VISUAL_DEBUG
