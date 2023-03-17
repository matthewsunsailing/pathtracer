#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.


  return true;

}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.

  // quadratic coeffs
  double a = (r.d).norm2();
  double b = 2 * dot(r.o - o, r.d);
  double c = (r.o - o).norm2() - r2;

  // b^2 - 4ac
  double discrim = b * b - 4 * a * c;

  // check for imaginary
  double t1 = r.max_t + 1;
  double t2 = r.min_t - 1;
  if (discrim >= 0) {
	t1 = (-b + sqrt(discrim)) / (2 * a);
	t2 = (-b - sqrt(discrim)) / (2 * a);
  }

  return ((t2 >= r.min_t) && (t2 <= r.max_t)) || ((t1 >= r.min_t) && (t1 <= r.max_t));
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.

  // quadratic coeffs
  double a = (r.d).norm2();
  double b = 2 * dot(r.o - o, r.d);
  double c = (r.o - o).norm2() - r2;

  // b^2 - 4ac
  double discrim = b * b - 4 * a * c;

  // check for imaginary
  double t1 = r.max_t + 1;
  double t2 = r.min_t - 1;

  double isect;
  bool isIsect = has_intersection(r);

  if (discrim >= 0) {
	t1 = (-b + sqrt(discrim)) / (2 * a);
	t2 = (-b - sqrt(discrim)) / (2 * a);

	if (isIsect) {
	  if (t2 >= r.min_t && t2 <= r.max_t) {
		isect = t2;
	  } else if (t1 >= r.min_t && t1 <= r.max_t) {
		isect = t1;
	  }
	  r.max_t = isect;

	  i->t = isect;
	  i->n = (r.o + isect * r.d - o);
	  i->n /= i->n.norm();
	  i->primitive = this;
	  i->bsdf = get_bsdf();
	}
	
  }

  return isIsect;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
