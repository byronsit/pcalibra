//
// Created by bohuan on 19-6-28.
//

#ifndef PCALIBRA_GROUND_EXTRACTION_H
#define PCALIBRA_GROUND_EXTRACTION_H

#include "pcalibra/utility.h"
#include "pcalibra/ground-extraction.h"


namespace pcalibra {
/**
 * @brief 不考虑时间的简单地面提取算法*/
class GroundExtraction {
 public:
  void Extract(const PointCloud3D &pointcloud, PointCloud3D * output);

  void Extract(const PointCloud3DI &pointcloud, PointCloud3D * output){
    PointCloud3D pc3d(pointcloud);
    Extract(pc3d, output);
  }


};

}//namespace

#endif //PCALIBRA_GROUND_EXTRACTION_H
