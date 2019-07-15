//
// Created by bohuan on 19-7-3.
//

#include "pcalibra/pillar-extraction.h"

namespace pcalibra{

/**
 * @brief 简单的过滤所有intensity小于ARG0的点，以及nan的点*/
void PillarExtraction::FilterByIntensity(const pcl::pclpc3i &pointcloud, pcl::pclpc3 *output) {
  for (const auto &it : pointcloud) {
    if (isnan(it.x) || it.intensity < ARG0) {
      continue;
    }
    pcl::PointXYZ xyz;
    xyz.x = it.x;
    xyz.y = it.y;
    xyz.z = it.z;
    output->push_back(xyz);
  }
}


/**
 * @brief check这个聚类是否是合法的
 * 1、求x,y,z的协方差矩阵)
 * 2、求协方差矩阵特征向量
 * 3、判断第二，第三大的值的情况，和最大的特征值做比较
 * @param pointcloud 输入点云
 * @parm output 输出的拟合的线段
 * @return true拟合成功，false拟合失败
 * @todo 利用cluster判断是否合法*/
bool PillarExtraction::CheckCluster(const pcl::pclpc3 &pointcloud, ELine *output){
  Eigen::Vector3d v;
  Eigen::Matrix3d r;
  v.setZero();
  for (auto it : pointcloud.points) {
    v.x() += it.x;
    v.y() += it.y;
    v.z() += it.z;
  }
  r = v * v.transpose();
  Eigen::Vector3cd t = r.eigenvalues();
  std::vector<double> res;
  res.push_back(fabs(double(t.x().real())));
  res.push_back(fabs(double(t.y().real())));
  res.push_back(fabs(double(t.z().real())));
  sort(res.begin(), res.end());
  //TODO 简单的根据特征向量判定一下
  {
    //TODO
  }
  FitLine fitline;
  fitline.SetInputPointCloud(pointcloud);
  fitline.Fit();
  fitline.GetResultLine(output); //拟合直线
  return true;
}

/**
 * @brief
 * 1、聚类，按照2D搜索半径简单的聚类。
 * 2、对每一类进行拟合直线，判断直线是否合法。
 * @param pointcloud 输入点云
 * @param output 输出的点云
 * */
void PillarExtraction::foo(const pcl::pclpc3 &pointcloud){
  pcl::pclpc3::Ptr points(new pcl::pclpc3(pointcloud));
  for (auto& it : *points ){
    it.z = 0;
  }
  //因为已经全部z坐标都归0了，所以不用担心3D最近点的问题
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster;
  std::vector<pcl::PointIndices> indices;
  cluster.setInputCloud(points);
  cluster.setMinClusterSize(ARG2);
  cluster.setClusterTolerance(ARG1);
  cluster.extract(indices);
  std::cout<<"the number of pillars is:" << indices.size() << std::endl;

  visualize(pointcloud);
  for (const auto &it : indices) {
    pcl::pclpc3 pc;
    ELine line;
    for (auto index : it.indices) {
      auto &xyz = pointcloud.points.at(index);
      pc.push_back(xyz);
    }
    if (CheckCluster(pc, &line)){
      lines_.push_back(line);
      pcs_.push_back(pc);
    }
  }
}



}//namespace
