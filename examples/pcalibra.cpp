//
// Created by bohuan on 19-6-27.
//

//#include <opencv2/opencv.hpp>
#include <pcalibra/ground-extraction.h>
#include "pcalibra/utility.h"
#include "pcalibra/pcl-pcalibra.h"
#include "pcalibra/core.h"

pcalibra::PointCloudLoader loader;
pcalibra::Core  core;

int main(){
 // std::cout<<"opencv version is :"<<CV_VERSION << std::endl;
  loader.Load("/media/bohuan/data/dataset/guangzhou/10-10-2/first.asc");//reference
  //pcalibra::PointCloud3DI pc3di_reference;
  std::vector<pcl::pclpc3i> t;
  //pcalibra::PointCloud3DI pc3di_aligned;
  pcalibra::PointCloud3D output;
  loader.Load("/media/bohuan/data/dataset/guangzhou/10-10-2/second_moved.asc");//aligned
  loader.GetPointCloud(&t);
  pcl::pclpc3i pc3di_reference = t[0];
  pcl::pclpc3i pc3di_aligned = t[1];
  core.SetAlignedPointCloud(pc3di_aligned);
  core.SetReferencePointCloud(pc3di_reference);
  //core.Align();//穷举匹配，比较慢
  core.AlignLines();//有很好的初值的话，则用和这个。TODO 改为输入初值




  return 0;
}