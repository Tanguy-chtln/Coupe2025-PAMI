// #include <Arduino.h>
#include <math.h>
#include <stdio.h>
#define PI 3.14159265359
#define TICKS_PER_ROTATION 250
typedef enum {
    LEFT = 0,
    RIGHT = 1
} DIRECTIONS;

typedef struct {
    DIRECTIONS motorId;
    float v;
    int ticks;
    float lastUpdateTime; // in ms 
} Motor;

typedef struct {
    float x; // x posititon
    float y; // y posititon
    float theta; // orientation
    const int id; // pami id
    const float e; // Space between left and right motors 
    const float r; // Radius of the wheels 
    Motor motors[2];
} PAMI;


float ticks_to_dist(int ticks, PAMI *pami) {
    return ticks * pami->r / TICKS_PER_ROTATION;
}

static const int kP=100;
static const int kI=10;
static const int kD=50;
int ComputePID(int error)
{

    static int lastError=0;
    static int errSum=0;

    int P,I,D;

    errSum += error;                 //Somme les erreurs depuis le début 

    int errDif = error - lastError;      //Calcule la variation de l'erreur
    lastError = error;

    P = error * kP;                  //Proportionnelle
    I = errSum * kI;                 //Intégrale
    D = errDif * kD;                 //Dérivée

    return P + I + D;                //Le résultat est la somme des trois
                                     //composantes calculées précédemment
}


void update_speed(Motor *motors); // Update motors speed using captors

float dist2(const float x0, const float y0, const float x1, const float y1) {
    return ((x1 -x0) * (x1 - x0)) + ((y1 - y0) * (y1 - y0));
}

void odometrie(float dt, PAMI *pami) {
    Motor *ml = pami->motors; // Motor left
    Motor *mr = pami->motors + 1; // Motor right
    const float d = (mr->v + ml->v)/2; // distance driven
    const float dtheta = (mr->v - ml->v) / 2; // Angle made
    
    // Update
    pami->theta += dtheta / pami->e;
    pami->x += d * cos(pami->theta);
    pami->y += d * sin(pami->theta);

    printf("Odométrie :\n");
    printf("dt : %f s\n", dt);
    printf("dtheta : %f\n", dtheta);
    printf("theta : %f\n", pami->theta);
    printf("x0, y0 : %f, %f\n", pami->x, pami->y);
    printf("\n");
}


void pami_to_position(float pamix, float pamiy, float pamia, float x, float y, float *d, float *a) {
    *a = (y - pamiy) / (x - pamix); // Angle between pami and destination
    *d = sqrtf(dist2(pamix, pamiy, x, y));
}

void setup()
{
    ;    //  Serial.begin(19200);
}





int main(int argc, char * argv[]) {
    Motor m1 = {LEFT, -0.2f, 0, 0};
    Motor m2 = {RIGHT, 0.2f, 0, 0};
    PAMI pami = {0, 0, PI/4, 0, 0.1, 0.023, {m1, m2}};
    odometrie(0.01, &pami);
    return 0;
}
