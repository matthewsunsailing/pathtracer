#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.

  // define normalized axis vector
  Vector3D xn(1, 0, 0);
  Vector3D yn(0, 1, 0);
  Vector3D zn(0, 0, 1);

  // yz
  double tx0 = dot(min - r.o, xn) / dot(r.d, xn);
  double tx1 = dot(max - r.o, xn) / dot(r.d, xn);
  if (tx1 < tx0) std::swap(tx0, tx1);

  // xz
  double ty0 = dot(min - r.o, yn) / dot(r.d, yn);
  double ty1 = dot(max - r.o, yn) / dot(r.d, yn);
  if (ty1 < ty0) std::swap(ty0, ty1);

  // xy
  double tz0 = dot(min - r.o, zn) / dot(r.d, zn);
  double tz1 = dot(max - r.o, zn) / dot(r.d, zn);
  if (tz1 < tz0) std::swap(tz0, tz1);

  // check rect intersection along 1 z-slice
  double tmin = std::max(tx0, ty0);
  double tmax = std::min(tx1, ty1);
  if (tmin > tmax) return false;

  // check rect intersection along z planes
  if (tz0 > tmax || tz1 < tmin) return false;
  tmin = std::max(tz0, tmin);
  tmax = std::min(tz1, tmax);

  // sanity check entry and exit
  if (tmin > tmax) return false;

  // check for overlap between ray and intersection intervals
  if (!((t1 - tmin) >= 0 && (tmax - t0) >= 0)) return false;

  t0, t1 = tmin, tmax;

  // otherwise,
  return true;

}

void BBox::draw(Color c, float alpha) const {

  glColor4f(c.r, c.g, c.b, alpha);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
