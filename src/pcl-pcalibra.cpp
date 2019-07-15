//
// Created by bohuan on 19-6-28.
//

#include "pcalibra/pcl-pcalibra.h"

namespace pcalibra{



void Pcalibra2PCL(const Point3I &point, pcl::PointXYZI * pcl_point) {
  pcl_point->x = point.x;
  pcl_point->y = point.y;
  pcl_point->z = point.z;
  pcl_point->intensity = point.intensity;
}

}//namespace