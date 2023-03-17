#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.


  BBox bbox;

  for (auto p = start; p != end; p++) {
    BBox bb = (*p)->get_bbox();
    bbox.expand(bb);
  }

  BVHNode *node = new BVHNode(bbox);
  
  int prim_ct = end - start;

  if (prim_ct <= max_leaf_size) {
    node->l = NULL;
    node->r = NULL;
    node->start = start;
    node->end = end;
  } else {
    Vector3D mean(0.0, 0.0, 0.0);

    for (auto p = start; p != end; p++) {
      mean += (*p)->get_bbox().centroid();
    }
    mean /= (float) prim_ct;

    int axis_split = 0;             // split axis of lowest SA
    float bbox_stat = INFINITY;     // split axis SA heuristic

    // split axis SA heuristic evaluation
    for (int axis = 0; axis < 3; axis++) {
      std::vector<Primitive*> lefts, rights;

      for (auto p = start; p != end; p++) {
        if ((*p)->get_bbox().centroid()[axis] <= mean[axis]) {
          lefts.push_back(*p);
        } else {
          rights.push_back(*p);
        }
      }

      BBox left_bbox;
      BBox right_bbox;

      for (int i = 0; i < lefts.size(); i++) {
        BBox bbox = lefts[i]->get_bbox();
        left_bbox.expand(bbox);
      }
      for (int j = 0; j < rights.size(); j++) {
        BBox bbox = rights[j]->get_bbox();
        right_bbox.expand(bbox);
      }

      Vector3D lex = left_bbox.extent;
      Vector3D rex = right_bbox.extent;
      float heur = lefts.size() * (lex.x * lex.y + lex.y * lex.z + lex.z * lex.x) +
                   rights.size() * (rex.x * rex.y + rex.y * rex.z + rex.z * rex.x);
      if (heur < bbox_stat) {
        bbox_stat = heur;
        axis_split = axis;
      }
    }

    // update pointers and recurse
    /*std::vector<Primitive*> lefts, rights;

    for (auto p = start; p != end; p++) {
      if ((*p)->get_bbox().centroid()[axis_split] <= mean[axis_split]) {
        lefts.push_back(*p);
      } else {
        rights.push_back(*p);
      }
    }

    auto cleft = start;
    auto cright = start;

    for (int i = 0; i < prim_ct; i++) {
      if (i < lefts.size()) {
        *cleft = lefts[i];
        cleft++;
      } else {
        *cright = rights[i - lefts.size()];
      }
      cright++;
    }*/

    int offset = 0;
    for (auto p = start; p != end; p++) {
      //in place swapping of the elemnts
      if ((*p)->get_bbox().centroid()[axis_split] < mean[axis_split]) {
        swap(*p, *(start + offset));
        offset += 1;
      }
    }

    node->l = construct_bvh(start, start + offset, max_leaf_size);
    node->r = construct_bvh(start + offset, end, max_leaf_size);
  }

  return node;


}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.

  double t0, t1;

  if (node->bb.intersect(ray, t0, t1)) {
    if (node->isLeaf()) {
      for (auto p = node->start; p != node->end; p++) {
        total_isects++;
        if ((*p)->has_intersection(ray)) {
          return true;
        }
      }
    } else {
      return has_intersection(ray, node->l) || has_intersection(ray, node->r);
    }
  } else {
    return false;
  }

  /*
  for (auto p : primitives) {
    total_isects++;
    if (p->has_intersection(ray))
      return true;
  }
  */

}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.

  double t0, t1;

  if (node->bb.intersect(ray, t0, t1)) {
    if (node->isLeaf()) {
      bool hit = false;
      for (auto p = node->start; p != node->end; p++) {
        total_isects++;
        hit = hit || (*p)->intersect(ray, i);
      }

      return hit;
    } else {
      bool lit = intersect(ray, i, node->l);
      bool rit = intersect(ray, i, node->r);
      return lit || rit;
    }
  }
  else {
    return false;
  }

  /*
  bool hit = false;
  for (auto p : primitives) {
    total_isects++;
    hit = p->intersect(ray, i) || hit;
  }
  return hit;
  */

}

} // namespace SceneObjects
} // namespace CGL
