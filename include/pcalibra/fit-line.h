/**
 * @pcl-fit-line.h
 * @brief 主要实现点云拟合一条直线的工作。
 * @author bohuan
 * @email bohuan.xue@gmail.com
 * @version 0.0.1
 * @date 2019-06-27
 * */
#ifndef PCALIBRA_FIT_LINE_H
#define PCALIBRA_FIT_LINE_H

#include "pcalibra/utility.h"
#include "pcalibra/pcl-pcalibra.h"
namespace pcalibra {

/**
 * @brief 传入点云传出直线
 * */
class FitLine {
 public:
  /**
   * @brief 塞入点云
   * @param source 输入的点云*/
  void SetInputPointCloud(pcl::pclpc3 source){
    pointcloud_ = source;
  }

  /**
   * @brief 主函数，开始自动拟合*/
  void Fit();

  /**
   * @brief 返回一条直线line
   * @param line 返回值*/
  void GetResultLine(ELine *line) {
    *line = result_;
  }

 private:
  //PointCloud3D pointcloud_;
  pcl::pclpc3 pointcloud_;
  ELine result_;



};
}

#endif //PCALIBRA_FIT_LINE_H
