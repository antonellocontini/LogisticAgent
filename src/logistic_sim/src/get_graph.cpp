#include "get_graph.hpp"

uint WIDTH_PX;
uint HEIGHT_PX;
float RESOLUTION = 1.0;
//  float WIDTH_M;
//  float HEIGHT_M;
float OFFSET_X;
float OFFSET_Y;

uint GetGraphDimension(const char *graph_file)
{

  FILE *file;
  file = fopen(graph_file, "r");
  uint dimension;

  if (file == NULL)
  {
    ROS_INFO("Can not open filename %s", graph_file);
    ROS_BREAK();
  }
  else
  {
    ROS_INFO("Graph File Opened. Reading Dimensions.\n");
    int r;
    r = fscanf(file, "%u", &dimension);

    //Initialize other dimension variables:
    r = fscanf(file, "%u", &WIDTH_PX);
    r = fscanf(file, "%u", &HEIGHT_PX);
    r = fscanf(file, "%f", &RESOLUTION);
    r = fscanf(file, "%f", &OFFSET_X);
    r = fscanf(file, "%f", &OFFSET_Y);
    //WIDTH_M = (float) WIDTH_PX * RESOLUTION;
    //HEIGHT_M = (float) HEIGHT_PX * RESOLUTION;

  }
  fclose(file);
  return dimension;
}

void GetGraphInfo(vertex *vertex_web, uint dimension, const char *graph_file)
{

  FILE *file;
  file = fopen(graph_file, "r");

  if (file == NULL)
  {
    ROS_INFO("Can not open filename %s", graph_file);
    ROS_BREAK();
  }
  else
  {
    ROS_INFO("Graph File Opened. Getting Graph Info.\n");

    uint i, j;
    float temp;
    int r;

    //Start Reading the File from FIRST_VID On:
    for (i = 0; i < FIRST_VID - 1; i++)
    {
      r = fscanf(file, "%f", &temp);
    }

    for (i = 0; i < dimension; i++)
    {

      r = fscanf(file, "%u", &vertex_web[i].id);

      r = fscanf(file, "%f", &vertex_web[i].x);
      vertex_web[i].x *= RESOLUTION; //convert to m
      vertex_web[i].x += OFFSET_X;

      r = fscanf(file, "%f", &vertex_web[i].y);
      vertex_web[i].y *= RESOLUTION; //convert to m
      vertex_web[i].y += OFFSET_Y;

      r = fscanf(file, "%u", &vertex_web[i].num_neigh);

      printf("cpoint( pose [%f %f 0 0] name \"point%u\" color \"black\") \n", vertex_web[i].x, vertex_web[i].y, vertex_web[i].id);

      for (j = 0; j < vertex_web[i].num_neigh; j++)
      {
        r = fscanf(file, "%u", &vertex_web[i].id_neigh[j]);
        r = fscanf(file, "%s", vertex_web[i].dir[j]);
        r = fscanf(file, "%u", &vertex_web[i].cost[j]); //can eventually be converted to meters also...
      }
    }
  }

  // printf("[v=10], x = %f (meters)\n", vertex_web[10].x);

  fclose(file);
}

uint IdentifyVertex(vertex *vertex_web, uint size, double x, double y)
{

  uint i, v = 0;
  double dif_x, dif_y, result = INFINITY;

  for (i = 0; i < size; i++)
  {
    dif_x = vertex_web[i].x - x;
    dif_y = vertex_web[i].y - y;

    //printf("[%u] result = %f, (dif_x+dif_y) = %f\n",i,result, fabs(dif_x) + fabs(dif_y));
    if (result > fabs(dif_x) + fabs(dif_y))
    { //Identify the Vertex closer to the initial coordinates x & y
      result = fabs(dif_x) + fabs(dif_y);
      v = i;
    }
  }
  return v;
}

uint GetNumberEdges(vertex *vertex_web, uint dimension)
{

  uint result = 0;

  for (uint i = 0; i < dimension; i++)
  {
    for (uint j = 0; j < vertex_web[i].num_neigh; j++)
    {
      if (vertex_web[i].id < vertex_web[i].id_neigh[j])
      {
        result++;
      }
    }
  }

  return result;
}


bool RemoveEdge_impl (vertex *vertex_web, uint dimension, uint u, uint v)
{
  // search vertex u
  bool good = false;
  uint u_index;
  for (uint u_index=0; u_index<dimension && !good; u_index++)
  {
    if (vertex_web[u_index].id == u)
    {
      good = true;
      break;
    }
  }

  if (!good)
  {
    // missing vertex with id u
    return false;
  }

  // search edge (u,v) among vertex u neighbours
  uint i;
  good = false;
  for (i=0; i<vertex_web[u_index].num_neigh && !good; i++)
  {
    if (vertex_web[u_index].id_neigh[i] == v)
    {
      good = true;
      vertex_web[u_index].num_neigh--;
      break;
    }
  }

  // edge is missing
  if (!good)
  {
    return false;
  }

  for (; i<vertex_web[u_index].num_neigh; i++)
  {
    vertex_web[u_index].id_neigh[i] = vertex_web[u_index].id_neigh[i+1];
    vertex_web[u_index].cost[i] = vertex_web[u_index].cost[i+1];
    vertex_web[u_index].cost_m[i] = vertex_web[u_index].cost_m[i+1];
    vertex_web[u_index].visited[i] = vertex_web[u_index].visited[i+1];
    vertex_web[u_index].dir[i][0] = vertex_web[u_index].dir[i+1][0];
    vertex_web[u_index].dir[i][1] = vertex_web[u_index].dir[i+1][1];
    vertex_web[u_index].dir[i][2] = vertex_web[u_index].dir[i+1][2];
  }

  return true;
}


bool RemoveEdge (vertex *vertex_web, uint dimension, uint u, uint v)
{
  // if (dimension <= u || dimension <= v)
  // {
  //   // invalid vertices ids
  //   return false;
  // }

  if (!RemoveEdge_impl(vertex_web, dimension, u, v))
  {
    return false;
  }
  return RemoveEdge_impl(vertex_web, dimension, v, u);
}


bool AddEdge_impl (vertex *vertex_web, uint dimension, uint u, uint v, uint cost)
{
  // search vertex u
  bool good = false;
  uint u_index;
  for (uint u_index=0; u_index<dimension && !good; u_index++)
  {
    if (vertex_web[u_index].id == u)
    {
      good = true;
      break;
    }
  }

  if (!good)
  {
    // missing vertex with id u
    return false;
  }

  // search edge (u,v) among vertex u neighbours
  uint i;
  for (i=0; i<vertex_web[u_index].num_neigh && good; i++)
  {
    if (vertex_web[u_index].id_neigh[i] == v)
    {
      good = false;
      break;
    }
  }

  // edge is already present
  if (!good)
  {
    return false;
  }

  // add edge
  uint num_neigh = vertex_web[u_index].num_neigh;
  if (num_neigh > 7)
  {
    return false;
  }
  vertex_web[u_index].id_neigh[num_neigh] = v;
  vertex_web[u_index].cost[num_neigh] = cost;
  vertex_web[u_index].visited[num_neigh] = false;
  vertex_web[u_index].num_neigh++;
  return true;
}

bool AddEdge (vertex *vertex_web, uint dimension, uint u, uint v, uint cost)
{
  if (!AddEdge_impl(vertex_web, dimension, u, v, cost))
  {
    return false;
  }
  return AddEdge_impl(vertex_web, dimension, v, u, cost);
}


//integer to array (itoa for linux c)
char *itoa(int value, char *str, int radix)
{
  static char dig[] =
      "0123456789"
      "abcdefghijklmnopqrstuvwxyz";
  int n = 0, neg = 0;
  unsigned int v;
  char *p, *q;
  char c;

  if (radix == 10 && value < 0)
  {
    value = -value;
    neg = 1;
  }
  v = value;
  do
  {
    str[n++] = dig[v % radix];
    v /= radix;
  } while (v);
  if (neg)
    str[n++] = '-';
  str[n] = '\0';

  for (p = str, q = p + (n - 1); p < q; ++p, --q)
    c = *p, *p = *q, *q = c;
  return str;
}