//
// Created by bohuan on 19-6-27.
//

#include <opencv2/opencv.hpp>
#include <pcalibra/ground-extraction.h>
#include "pcalibra/utility.h"
#include "pcalibra/pcl-pcalibra.h"
#include "pcalibra/core.h"

pcalibra::PointCloudLoader loader;
pcalibra::Core  core;

int main(){
  std::cout<<"opencv version is :"<<CV_VERSION << std::endl;
  loader.LoadVelodyne("/media/bohuan/data/dataset/pcalibra/a/a.asc");
  //pcalibra::PointCloud3DI pc3di_reference;
  std::vector<pcl::pclpc3i> t;
  //pcalibra::PointCloud3DI pc3di_aligned;
  pcalibra::PointCloud3D output;
  loader.LoadVelodyne("/media/bohuan/data/dataset/pcalibra/b/bb.asc");
  loader.GetPointCloud(&t);
  pcl::pclpc3i pc3di_reference = t[0];
  pcl::pclpc3i pc3di_aligned = t[1];
  core.SetAlignedPointCloud(pc3di_aligned);
  core.SetReferencePointCloud(pc3di_reference);
  core.AlignLines();




  return 0;
}