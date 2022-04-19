#include "global.h"
#include "utils.h"


char *pathToCenterFrom00;
char *pathToCenterFrom06;
char *pathToCenterFrom60;
char *pathToCenterFrom66;

char *pathToFollow;
int turnArray[50];

void NAVCOMP_MODULE();
int transformStringToPath();
void followPath(int);
void updateLocation();
void adjustYaw();
double adjustPath();
void exploreLabyrinth(int, int);