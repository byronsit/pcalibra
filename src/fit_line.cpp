//
// Created by bohuan on 19-6-27.
//

#include "../include/pcalibra/fit-line.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>   //随机参数估计方法头文件
#include <pcl/sample_consensus/model_types.h>   //模型定义头文件
#include <pcl/segmentation/sac_segmentation.h>   //基于采样一致性分割的类的头文件



namespace pcalibra{

void Fit2(const pcl::pclpc3 &pc, ELine *result){
  Eigen::Matrix<double_t ,Eigen::Dynamic, Eigen::Dynamic> m(3, pc.size());
  double sx(0), sy(0), sz(0);
  for (int i = 0; i < pc.size(); ++ i) {
    m(0,i) = pc.at(i).x;
    m(1,i) = pc.at(i).y;
    m(2,i) = pc.at(i).z;
    sx += pc.at(i).x;
    sy += pc.at(i).y;
    sz += pc.at(i).z;
  }
  sx = sx / pc.size();
  sy = sy / pc.size();
  sz = sz / pc.size();
  Eigen::Matrix<double_t, Eigen::Dynamic, Eigen::Dynamic> q(3, pc.size());
  for (int i =0; i < pc.size(); ++ i){
    q(0,i) = pc.at(i).x - sx;
    q(1,i) = pc.at(i).y - sy;
    q(2,i) = pc.at(i).z - sz;
  }
  auto X = q*q.transpose();
  Eigen::JacobiSVD<Eigen::Matrix<double_t, Eigen::Dynamic, Eigen::Dynamic> > svd(q, Eigen::ComputeFullV | Eigen::ComputeFullU); // ComputeThinU | ComputeThinV
  auto U = svd.matrixU();
  //std::cout << U << std::endl;
  result->vector.x() = U(0,0);
  result->vector.y() = U(1,0);
  result->vector.z() = U(2,0);
  result->start.x() = sx;
  result->start.y() = sy;
  result->start.z() = sz;
  std::cout << result->vector << std::endl << std::endl;
  sleep(1);
}

void FitLine::Fit(){
  Fit2(pointcloud_, &result_);
  result_.AdjustFromStartVector(); //通过向量得到start和end的坐标
  return;
  //PointCloud3D result;
  //cv::fitLine(pointcloud_.points, result.points, cv::DIST_L2, 0, 0.01, 0.01);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true); //干啥的？
  seg.setModelType(pcl::SACMODEL_LINE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(10000);
  seg.setDistanceThreshold(0.01);
  pcl::pclpc3::Ptr pc(new pcl::pclpc3(pointcloud_));
  seg.setInputCloud(pc);
  seg.segment(*inliers, *coefficients);

  result_.vector.x() = coefficients->values.at(3);
  result_.vector.y() = coefficients->values.at(4);
  result_.vector.z() = coefficients->values.at(5);
  result_.start.x() =  coefficients->values.at(0);
  result_.start.y() =  coefficients->values.at(1);
  result_.start.z() =  coefficients->values.at(2);
  result_.AdjustFromStartVector(); //通过向量得到start和end的坐标
  std::cout<<result_.vector << std::endl;
}

}
