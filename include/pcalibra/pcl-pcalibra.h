//
// Created by bohuan on 19-6-28.
//

#ifndef PCALIBRA_PCL_PCALIBRA_H
#define PCALIBRA_PCL_PCALIBRA_H


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "pcalibra/utility.h"


namespace pcl {

typedef pcl::PointCloud<pcl::PointXY> pclpc2;
typedef pcl::PointCloud<pcl::PointXYZ> pclpc3;
typedef pcl::PointCloud<pcl::PointXYZI> pclpc3i;
typedef pcl::PointCloud<pcl::PointXYZINormal> pclpc3in;
typedef pcl::PointCloud<pcl::Normal> pclpcn;

typedef pcl::KdTreeFLANN<pcl::PointXY, ::flann::L1<float>> Manhattan2DKdTree_T;//貌似不对
typedef pcl::KdTreeFLANN<pcl::PointXY, ::flann::L2_Simple<float>> Euclidean2DKdTree_T;
typedef pcl::KdTreeFLANN<pcl::PointXYZ, ::flann::L2_Simple<float>> Euclidean3DKdTreeIn2D_T;
//typedef pcl::KdTreeFLANN<pcl::PointXY> Manhattan2DKdTree_T;
typedef pcl::KdTreeFLANN<pcl::PointXYZ> KdTree_T;

/**
 * @brief 求解pcl的向量的norm
 * @todo 没写呢*/
//void norm(){}
} // namespace

namespace pcalibra {


template <typename _Pt, typename _Tpcl>
void Pcalibra2PCL(PointCloud3D_<_Pt> pointcloud, pcl::PointCloud<_Tpcl>);

template<typename _Tpcl>
void Pcalibra2PCL(const Point3 &point, _Tpcl * pcl_point);

void Pcalibra2PCL(const Point3I &point, pcl::PointXYZI * pcl_point);


} //namespace



/*这里主要是上面声明的函数的部分实现，因为模板的原因*/
namespace pcalibra{

template <typename _Pt, typename _Tpcl> inline
void Pcalibra2PCL(const PointCloud3D_<_Pt> &pointcloud, pcl::PointCloud<_Tpcl>* pcl_pointcloud){
  pcl_pointcloud->clear();
  for (const auto &it : pointcloud.points) {
    _Tpcl xyz;
    xyz.x = it.x;
    xyz.y = it.y;
    xyz.z = it.z;
    pcl_pointcloud->push_back(xyz);
  }
}

template<typename _Tpcl> inline
void Pcalibra2PCL(const Point3 &point, _Tpcl * pcl_point){
  pcl_point->x = point.x;
  pcl_point->y = point.y;
  pcl_point->z = point.z;
}

} //namespace

#endif //PCALIBRA_PCL_PCALIBRA_H
