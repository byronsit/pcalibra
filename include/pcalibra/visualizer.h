//
// Created by bohuan on 19-7-4.
//

#ifndef PCALIBRA_VISUALIZER_H
#define PCALIBRA_VISUALIZER_H

#include "pcalibra/pcl-pcalibra.h"
#include <pcl/visualization/pcl_visualizer.h>


namespace pcalibra {

static void visualize(const pcl::pclpc3 &p){
  pcl::visualization::PCLVisualizer viewer("viewer");
  viewer.setBackgroundColor(0, 0, 0);
  pcl::pclpc3::Ptr t(new pcl::pclpc3(p));
  viewer.addPointCloud<pcl::PointXYZ>(t, "sample cloud");
  while (!viewer.wasStopped()) {
    viewer.spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(1000));
  }
}

} //namespace

#endif //PCALIBRA_VISUALIZER_H
