#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <ros/ros.h>

#include "struct_vertex.hpp"

extern uint WIDTH_PX;
extern uint HEIGHT_PX;
extern float RESOLUTION;
// extern float WIDTH_M;
// extern float HEIGHT_M;
extern float OFFSET_X;
extern float OFFSET_Y;

uint GetGraphDimension (const char* graph_file);

void GetGraphInfo (vertex *vertex_web, uint dimension, const char* graph_file);
  
uint IdentifyVertex (vertex *vertex_web, uint size, double x, double y);

uint GetNumberEdges (vertex *vertex_web, uint dimension);
  
//integer to array (itoa for linux c)
char* itoa(int value, char* str, int radix);

#include "impl/get_graph.i.hpp"