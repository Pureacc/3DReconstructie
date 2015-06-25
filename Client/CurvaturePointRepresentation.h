#ifndef __CURVATUREPOINTREPRESENTATION_H_INCLUDED__
#define __CURVATUREPOINTREPRESENTATION_H_INCLUDED__

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

class CurvaturePointRepresentation : public pcl::PointRepresentation <pcl::PointNormal> {
  using pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_;

public:
  CurvaturePointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const pcl::PointNormal &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};

#endif