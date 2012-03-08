/** /author Andrew Dornbrush */

#include <sbpl_arm_planner/sbpl_geometry_utils.h>

void sbpl_geometry_utils::getEnclosingSpheresOfCube(double xSize, double ySize, double zSize, double radius, std::vector<sbpl_geometry_utils::Sphere> &spheres)
{
  spheres.clear();
  sbpl_geometry_utils::Sphere s; s.radius = radius;
  double rootTwo = sqrt(2.0);
  double enclCubeLength = rootTwo * radius;

  int xNumSpheres = ceil(xSize / (enclCubeLength));
  int yNumSpheres = ceil(ySize / (enclCubeLength));
  int zNumSpheres = ceil(zSize / (enclCubeLength));

  // Compute the coordinate of the sphere in the bottom-left-back corner
  double xStart = -1.0 * (xNumSpheres / 2) * enclCubeLength;
  if (xNumSpheres % 2 == 0) xStart += enclCubeLength / 2.0;

  double yStart = -1.0 * (yNumSpheres / 2) * enclCubeLength;
  if (yNumSpheres % 2 == 0) yStart += enclCubeLength / 2.0;

  double zStart = -1.0 * (zNumSpheres / 2) * enclCubeLength;
  if (zNumSpheres % 2 == 0) zStart += enclCubeLength / 2.0;

  // Compute the locations of all the spheres
  for (int x = 0; x < xNumSpheres; x++) {
    for (int y = 0; y < yNumSpheres; y++) {
      for (int z = 0; z < zNumSpheres; z++) {
        s.p.x = xStart + enclCubeLength * x;
        s.p.y = yStart + enclCubeLength * y;
        s.p.z = zStart + enclCubeLength * z;
        spheres.push_back(s);
      }
    }
  }
}

void sbpl_geometry_utils::getEnclosingSpheresOfCube(double xSize, double ySize, double zSize, double radius, std::vector<std::vector<double> > &spheres)
{
  std::vector<sbpl_geometry_utils::Sphere> s;
  getEnclosingSpheresOfCube(xSize, ySize, zSize, radius, s);

  spheres.resize(s.size(), std::vector<double> (4,0));
  for(std::size_t i = 0; i < s.size(); ++i)
  {
    spheres[i][0] = s[i].p.x;
    spheres[i][1] = s[i].p.y;
    spheres[i][2] = s[i].p.z;
    spheres[i][3] = s[i].radius;
  }
}

