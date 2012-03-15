/** /author Andrew Dornbush */

#ifndef _SBPL_GEOMETRY_UTILS_
#define _SBPL_GEOMETRY_UTILS_

#include <vector>
#include <math.h>

namespace sbpl_geometry_utils
{

struct Point
{
  double x;
  double y;
  double z;
};

struct Sphere
{
  Point p;
  double radius;
};

void getEnclosingSpheresOfCube(double xSize, double ySize, double zSize, double radius, std::vector<Sphere>& spheres);

void getEnclosingSpheresOfCube(double xSize, double ySize, double zSize, double radius, std::vector<std::vector<double> >& spheres);

}
#endif
