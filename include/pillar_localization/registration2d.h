//
// Created by bohuan on 19-5-17.
//

#ifndef PILLAR_LOCALIZATION_REGISTRATION2D_H
#define PILLAR_LOCALIZATION_REGISTRATION2D_H


#include <pcl/registration/icp.h>
#include <pcl/registration/ndt_2d.h>
#include "pillar_localization/registration.h"



namespace {
void Points2dto3d(const PointCloud2D_T &input, PointCloud_T::Ptr output){
  output->clear();
  for (auto it : input) {
    pcl::PointXYZ xyz;
    xyz.x = it.x;
    xyz.y = it.y;
    xyz.z = 0;
    output->push_back(xyz);
  }
}

void Points3dto2d(const PointCloud_T &input, PointCloud2D_T::Ptr output){
  output->clear();
  for (auto it : input) {
    pcl::PointXY xy;
    xy.x = it.x;
    xy.y = it.y;
    output->push_back(xy);
  }
}

}

namespace plz {
class Registration2d : public Registration<PointCloud2D_T> {
 public:

  virtual void SetParameter(const std::string parameter) override {
    //TODO
  }

  virtual void ComputeTransformation (PointCloud2D_T::Ptr output_ptr, Eigen::Matrix4f *result_ptr) override;

};
}

#endif //PILLAR_LOCALIZATION_REGISTRATION2D_H
