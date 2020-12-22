#pragma once 

//File Line of the First Vertex ID to read (Protection) - fscanf() ignores blank lines
#define FIRST_VID 7

using uint = unsigned int;

extern uint WIDTH_PX;
extern uint HEIGHT_PX;
extern float RESOLUTION;
extern float OFFSET_X;
extern float OFFSET_Y;

struct vertex{
    uint id, num_neigh;
    float x,y;
    uint id_neigh[8], cost[8];
    double cost_m[8];
    bool visited[8];
    char dir[8][3];
};
