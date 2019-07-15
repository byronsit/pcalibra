//
// Created by bohuan on 19-6-30.
//

#include "pcalibra/fit-plane.h"
#include "pcalibra/pcl-pcalibra.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>   //随机参数估计方法头文件
#include <pcl/sample_consensus/model_types.h>   //模型定义头文件
#include <pcl/segmentation/sac_segmentation.h>   //基于采样一致性分割的类的头文件


namespace pcalibra{

void FitPlane::SetInputPointCloud(pcalibra::PointCloud3D source) {
  this->pointcloud_ = source;
}

void FitPlane::Fit() {
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  //std::vector<int> indices;
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true); //干啥的？
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(10000);
  seg.setDistanceThreshold(0.01);
  pcl::pclpc3::Ptr pc(new pcl::pclpc3);
  Pcalibra2PCL(this->pointcloud_, pc.get());
  seg.setInputCloud(pc);
  seg.segment(*inliers, *coefficients);
  std::cout<<"fit line coefficients:";
  for (int i = 0; i < coefficients->values.size(); ++ i) {
    std::cout<<coefficients->values.at(i)<<" ";
    this->result_.coefficients.at(i) = coefficients->values.at(i);

  }
  std::cout<<std::endl;
}

}//namespace

