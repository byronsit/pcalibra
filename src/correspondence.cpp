//
// Created by bohuan on 19-6-30.
//

#include "pcalibra/correspondence.h"

namespace pcalibra{

Correspondence::Correspondence(const ELine &line, const ELine &line2):corres_type(Line2Line) {
  this->line_ = line;
  this->line2_ = line2;
}

Correspondence::Correspondence(const Line3 &line, const Line3 &line2):corres_type(Line2Line) {
  line_.vector[0] = line.vector.x;
  line_.vector[1] = line.vector.y;
  line_.vector[2] = line.vector.z;
  line_.start[0] = line.start.x;
  line_.start[1] = line.start.y;
  line_.start[2] = line.start.z;
  line_.end[0] = line.end.x;
  line_.end[1] = line.end.y;
  line_.end[2] = line.end.z;

  line2_.vector[0] = line2.vector.x;
  line2_.vector[1] = line2.vector.y;
  line2_.vector[2] = line2.vector.z;
  line2_.start[0]  = line2.start.x;
  line2_.start[1]  = line2.start.y;
  line2_.start[2]  = line2.start.z;
  line2_.end[0]    = line2.end.x;
  line2_.end[1]    = line2.end.y;
  line2_.end[2]    = line2.end.z;
}

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
