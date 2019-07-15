//
// Created by bohuan on 19-5-17.
//

#ifndef PILLAR_LOCALIZATION_REGISTRATION_H
#define PILLAR_LOCALIZATION_REGISTRATION_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud_T;
typedef pcl::PointCloud<pcl::PointXY> PointCloud2D_T;


namespace plz{
template<typename PointType>
class Registration {
 public:
  void SetSourcePoints(typename PointType::Ptr source_points_ptr) {
    source_points_ptr_ = source_points_ptr;
  }

  void SetTargetPoints(typename PointType::Ptr target_points_ptr) {
    target_points_ptr_= target_points_ptr;
  }

  virtual void SetParameter(const std::string parameter) = 0;
  virtual void ComputeTransformation (typename PointType::Ptr output_ptr, Eigen::Matrix4f *result_ptr) = 0;

 protected:
  typename PointType::Ptr source_points_ptr_;
  typename PointType::Ptr target_points_ptr_;


};
} //namespace

#endif //PILLAR_LOCALIZATION_REGISTRATION_H
