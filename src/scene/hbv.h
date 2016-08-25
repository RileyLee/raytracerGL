#ifndef __HBV_H__
#define __HBV_H__

#include "scene.h"

extern bool debugMode;

std::ostream &operator<<(std::ostream &str, const BoundingBox &bbox) {
  str << "[Min: " << bbox.min << ", Max: " << bbox.max << "]";
  return str;
}

class HBV {
private:
  class HBV_Node {
  public:
	HBV_Node(Geometry *g) : bbox(g), left(NULL), right(NULL), isLeaf(true) { }
	HBV_Node(const BoundingBox &b, HBV_Node *l, HBV_Node *r) : bbox(new BoundingBox(b)), left(l), right(r), isLeaf(false) { }
	inline const BoundingBox &getBoundingBox() const {
	  if(isLeaf) {
		return reinterpret_cast<Geometry*>(bbox)->getBoundingBox();
	  } else {
		return *reinterpret_cast<BoundingBox*>(bbox);
	  }
	}
	bool intersect(const ray &r, isect &i) const {
	  const BoundingBox &b = getBoundingBox();
	  double tMin, tMax;
	  if(!b.intersect(r, tMin, tMax)) {
		if(debugMode && false) {
		  std::cout<< "does not interset bb, skipping " << b << std::endl;
		}
		return false;
	  }
	  if(debugMode && false) {
		std::cout << "SUCCESS! " << b << std::endl;
	  }
	  
	  if(!isLeaf) {
		isect rightHit, leftHit;
		bool rightFound = false, leftFound = false;
		if(left) {
		  leftFound = left->intersect(r, leftHit);
		}
		if(right) {
		  rightFound = right->intersect(r, rightHit);
		}
		if(leftFound && rightFound) {
		  if(leftHit.t < rightHit.t) {
			i = leftHit;
		  } else {
			i = rightHit;
		  }
		  return true;
		} else if(leftFound) {
		  i = leftHit;
		  return true;
		} else if(rightFound) {
		  i = rightHit;
		  return true;
		}
		return false;
	  } else {
		if(debugMode && false) {
		  std::cout << "and here we ARE! " << b << " " << std::endl;
		}
		Geometry *g = reinterpret_cast<Geometry*>(bbox);
		bool ret = g->intersect(r, i);
		if(debugMode && ret && false) {
		  std::cout << "hit " << b << std::endl;
		}
		return ret;
	  }
	}
	~HBV_Node() {
	  delete left;
	  delete right;
	  if(!isLeaf) {
		delete reinterpret_cast<BoundingBox*>(bbox);
	  }
	}
	void *bbox;
	HBV_Node *left, *right;
	bool isLeaf;
  };
  HBV_Node* root;
  HBV_Node *buildNode(const std::vector<Geometry*> &input, const BoundingBox& bbox, int axis) {
	if(input.size() == 0) {
	  return NULL;
	} else if(input.size() == 1) {
	  return new HBV_Node(input.at(0));
	} else if(input.size() == 2) {
	  return new HBV_Node(bbox, new HBV_Node(input.at(0)), new HBV_Node(input.at(1)));
	}
	bool leftInit = false;
	bool rightInit = false;
	Vec3d minLeft, minRight, maxLeft, maxRight;
	double midPoint = bbox.min[axis] + ((bbox.max[axis] - bbox.min[axis]) / 2);
	std::vector<Geometry*> left, right;
	//std::cout << "On axis: " << axis << " with midpoint: " << midPoint << " and bbox " << bbox << std::endl;
	// above, right, farther away is in the "right" bucket
	for(int i = 0; i < input.size(); i++) {
	  Geometry *g = input.at(i);
	  const BoundingBox& bbox = g->getBoundingBox();
	  if(bbox.min[axis] > midPoint) {
		//std::cout << "object with box " << bbox << " is right" << std::endl;
		right.push_back(g);
		updateBox(minRight, maxRight, bbox, rightInit);
		// straddling the plane or to the bottom/left/closer is in the left
	  } else {
		//std::cout << "object with box " << bbox << " is left" << std::endl;
		updateBox(minLeft, maxLeft, bbox, leftInit);
		left.push_back(g);
	  }
	}
	if(left.size() == 0 || right.size() == 0) {
	  left.clear();
	  right.clear();
	  int end = input.size() / 2;
	  leftInit = rightInit = false;
	  for(int i = 0; i < end; i++) {
		Geometry *g = input.at(i);
		const BoundingBox &bbox = g->getBoundingBox();
		updateBox(minLeft, maxLeft, bbox, leftInit);
		left.push_back(g);
	  }
	  for(int i = end; i < input.size(); i++) {
		Geometry *g = input.at(i);
		const BoundingBox &bbox = g->getBoundingBox();
		updateBox(minRight, maxRight, bbox, rightInit);
		right.push_back(g);
	  }
	}
	// leftIt should point to the index of the last geometry on the "left", and rightIt shoudl point to the 
	// index of the first geometry on the "right"
	//std::cout << "Using bounding box: " << minLeft << " -> " << maxLeft << std::endl;
	//std::cout << "Using bounding box: " << minRight << " -> " << maxRight << std::endl;
	HBV_Node *left_node = buildNode(left, BoundingBox(minLeft, maxLeft), (axis + 1) % 3);
	HBV_Node *right_node = buildNode(right, BoundingBox(minRight, maxRight), (axis + 1) % 3);
	return new HBV_Node(bbox, left_node, right_node);
  }
  inline void updateBox(Vec3d &min, Vec3d &max, const BoundingBox &bbox, bool &init) {
	if(init) {
	  min = minimum(min, bbox.min);
	  max = maximum(max, bbox.max);
	} else {
	  min = bbox.min;
	  max = bbox.max;
	  init = true;
	}
  }
public:
  HBV() : root(NULL) { }
  ~HBV() {
	delete root;
  }
  bool intersect(const ray& r, isect &i) const {
	assert(root != NULL);
	return root->intersect(r, i);
  }
  void build(const std::vector<Geometry*> &objects, const BoundingBox &sceneBox) {
	delete root;
	//std::cout << sceneBox.min << " " << sceneBox.max << std::endl;
	std::vector<Geometry*> partitionList(objects);
	root = buildNode(partitionList, sceneBox, 0);
  }
};

#endif
