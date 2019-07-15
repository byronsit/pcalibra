//
// Created by bohuan on 19-6-30.
//

#ifndef PCALIBRA_CORRESPONDENCE_H
#define PCALIBRA_CORRESPONDENCE_H

#include <ceres/jet.h>
#include "utility.h"
#include <glog/logging.h>

namespace pcalibra {

typedef enum CroresType {None, Point2Line, Point2Plane} CorresType;

/**
 * @brief 对应关系,点和线的关系。点和面的关系两种
 * 显然，要优化的都是point。*/
class Correspondence {
 public:

  CorresType corres_type = None; ///< 默认类型是没有类型

  Correspondence() = delete;

  Correspondence(const Correspondence &cor):point_(cor.point_), line_(cor.line_), plane_(cor.plane_), corres_type(cor.corres_type){}

  Correspondence(const EPoint &point, const ELine &line):point_(point),line_(line), corres_type(Point2Line){}

  Correspondence(const EPoint &point, const EPlane &plane):point_(point),plane_(plane), corres_type(Point2Plane){}

  Correspondence(const Point3 &point, const Line3 &line);

  Correspondence(const Point3 &point, const Plane &plane);

  /**
   * @brief 直接函数，点线距或者点面距
   * @param V [x y z]
   * @parm R[qx qy qz qw]
   * @return 对于corres_type没设定的情况，返回NAN
   * */
  template<typename T>
  T Function(Eigen::Map<const Eigen::Matrix<T, 3, 1> > &V, const Eigen::Map<const Eigen::Quaternion<T>> &R) const;


 private:
  EPoint point_;
  ELine line_;
  EPlane plane_;
};

}//namespace


namespace pcalibra{

template<typename T> inline
T Correspondence::Function(Eigen::Map<const Eigen::Matrix<T, 3, 1> > &V, const Eigen::Map<const Eigen::Quaternion<T>> &R) const{
  Eigen::Matrix<T, 3, 1> P(point_.cast<T>());
  P = (R * P + V);
  CHECK_NE(corres_type, None);
  if (corres_type == Point2Line) { //点线距
    Eigen::Matrix<T, 3, 1> p, t, r;
    p = line_.vector.cast<T>();
    t = P - line_.start.cast<T>();
    r = p.cross(t);
    //std::cout<<"["<<r<<"]"<<std::endl;
    return r.norm();
  }
  CHECK_EQ(corres_type, Point2Plane);
  if (corres_type == Point2Plane) { //点面距
    Eigen::Matrix<T, 3, 1> p, t ,r;
    p.setZero();
    p[2] = - plane_.coefficients.matrix().cast<T>()[3] / plane_.coefficients.matrix().cast<T>()[2];
    t = plane_.coefficients.matrix().cast<T>().block(0 ,0, 3, 1);
    r = point_ - p;
    return  r.dot(t);;
  }
}

}//namespace

#endif //PCALIBRA_CORRESPONDENCE_H
