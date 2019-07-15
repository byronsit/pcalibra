//
// Created by bohuan on 19-6-30.
//

#ifndef PCALIBRA_FIT_PLANE_H
#define PCALIBRA_FIT_PLANE_H

#include "pcalibra/utility.h"

namespace pcalibra {
class FitPlane {
 public:
   /**
   * @brief 塞入点云
   * @param source 输入的点云*/
  void SetInputPointCloud(PointCloud3D source);

  void Fit();





 private:
  PointCloud3D pointcloud_;
  Plane result_;
};
} //namespace

#endif //PCALIBRA_FIT_PLANE_H
