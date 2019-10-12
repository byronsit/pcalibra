
#include "../include/pcalibra/utility.h"

namespace pcalibra{

void PointCloudLoader::LoadVelodyne(const std::string &path) {
  FILE *fin = fopen(path.c_str(), "r");
  double x, y, z, intensity, line;
  pointcloud_.push_back(pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointXYZI t;
  while (~fscanf(fin, "%f %f %f %f %lf", &t.x, &t.y, &t.z, &t.intensity, &line)){
    pointcloud_.rbegin()->push_back(t);
  }
  fclose(fin);
}



void PointCloudLoader::Load(const std::string &path) {
  FILE *fin = fopen(path.c_str(), "r");
  pointcloud_.push_back(pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointXYZI t;
  while (~fscanf(fin, "%f %f %f %f", &t.x, &t.y, &t.z, &t.intensity)){
    pointcloud_.rbegin()->push_back(t);
  }
  fclose(fin);
}

void Line3::AdjustFromStartEnd() {
  //TODO
  exit(-1);

}

void Line3::AdjustFromVector() {
  if (true == FEQUAL(vector.z, 0)){
    DEBUG_OUT("the line is paralleled to the ground");
  }

  if (false == FEQUAL(norm(vector), 0)) {
    auto q = norm(vector);
    vector.x /= q;
    vector.y /= q;
    vector.z /= q;
  }

  double_t q = start.z / vector.z;
  start.x = start.x - q * vector.x;
  start.y = start.y - q * vector.x;
  start.z = start.z - q * vector.x;
  if (fabs(start.z) > FEPS) {
    DEBUG_OUT("fabs error");
  }
  end.x += vector.x;
  end.y += vector.y;
  end.z += vector.z;
}


void ELine::AdjustFromStartVector(){
  if (true == FEQUAL(vector.z(), 0)){
    DEBUG_OUT("the line is paralleled to the ground");
  }
  if (false == FEQUAL((vector.norm()), 1)) {
    auto q = vector.norm();
    vector.x() /= q;
    vector.y() /= q;
    vector.z() /= q;
  }
  double_t q = start.z() / vector.z();
  start.x() = start.x() - q * vector.x();
  start.y() = start.y() - q * vector.y();
  start.z() = start.z() - q * vector.z();
  if (fabs(start.z()) > FEPS) {
    DEBUG_OUT("fabs error");
  }
  end.x() += vector.x();
  end.y() += vector.y();
  end.z() += vector.z();
}

}//namespace
