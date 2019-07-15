//
// Created by bohuan on 19-7-3.
//

#ifndef PCALIBRA_PILLAR_EXTRACTION_H
#define PCALIBRA_PILLAR_EXTRACTION_H

#include "pcalibra/utility.h"
#include "pcalibra/pcl-pcalibra.h"
#include "pcalibra/parameter.h" ///<需要ARG0等参数
#include "pcalibra/fit-line.h"


#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/organized.h>
#include <pcl/search/impl/organized.hpp>


#include "pcalibra/visualizer.h"

namespace pcalibra {

/**
 * @brief 用来提取反光的柱子*/
class PillarExtraction {
 public:


  /**
   * @brief 从带intensity的点云里提取出所有的line
   * @param pointcloud 输入的intensity的点云
   * @param output 输出的若干条line*/
  void Extract(const pcl::pclpc3i &pointcloud) {
    lines_.clear();
    pcs_.clear();
    pcl::pclpc3 o;
    FilterByIntensity(pointcloud, &o);
    //OutputPoints(o,"/media/bohuan/data/dataset/pcalibra/a.txt");
    foo(o);
  }

  void GetLines(std::vector<ELine> *line){
    *line = lines_;
  }

  void GetPointCloud(std::vector<pcl::pclpc3> *pc){
    *pc = pcs_;
  }

 private:

  /**
   * @brief 简单的过滤所有intensity小于ARG0的点，以及nan的点*/
  void FilterByIntensity(const pcl::pclpc3i &pointcloud, pcl::pclpc3 *output);


  /**
   * @brief check这个聚类是否是合法的
   * 1、求x,y,z的协方差矩阵)
   * 2、求协方差矩阵特征向量
   * 3、判断第二，第三大的值的情况，和最大的特征值做比较
   * @param pointcloud 输入点云
   * @parm output 输出的拟合的线段
   * @return true拟合成功，false拟合失败*/
  bool CheckCluster(const pcl::pclpc3 &pointcloud, ELine *output);

  /**
   * @brief
   * 1、聚类，按照2D搜索半径简单的聚类。
   * 2、对每一类进行拟合直线，判断直线是否合法。
   * @param pointcloud 输入点云
   * @param output 输出的点云
   * */
  void foo(const pcl::pclpc3 &pointcloud);


 private:
  Line3 normal_; //地面的平均法向量
  std::vector<ELine> lines_; ///< 保存所有求出的棍子
  std::vector<pcl::pclpc3> pcs_;

};
}//namespace

#endif //PCALIBRA_PILLAR_EXTRACTION_H
