//
// Created by bohuan on 19-6-30.
//

#include "pcalibra/correspondence.h"

namespace pcalibra{


Correspondence::Correspondence(const Point3 &point, const Line3 &line):corres_type(Point2Line){
  point_[0] = point.x;
  point_[1] = point.y;
  point_[2] = point.z;
  line_.vector[0] = line.vector.x;
  line_.vector[1] = line.vector.y;
  line_.vector[2] = line.vector.z;
  line_.start[0] = line.start.x;
  line_.start[1] = line.start.y;
  line_.start[2] = line.start.z;
  line_.end[0] = line.end.x;
  line_.end[1] = line.end.y;
  line_.end[2] = line.end.z;
}

Correspondence::Correspondence(const Point3 &point, const Plane &plane):corres_type(Point2Plane){
  point_[0] = point.x;
  point_[1] = point.y;
  point_[2] = point.z;
  this->plane_.coefficients[0] = plane.a();
  plane_.coefficients[1] = plane.b();
  plane_.coefficients[2] = plane.c();
  plane_.coefficients[3] = plane.d();
}


}
