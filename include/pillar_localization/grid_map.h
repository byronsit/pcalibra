//
// Created by bohuan on 19-5-17.
//

#ifndef PILLAR_LOCALIZATION_GRID_MAP_H
#define PILLAR_LOCALIZATION_GRID_MAP_H

#include <iostream>
#include <limits>
#include <array>

namespace plz{

template <typename PointCloudType>
class Grid_Map {
 public:
  virtual void SetGlobalMap(const typename PointCloudType::Ptr &global_map_ptr, const float &resolution) = 0;
 protected:

  bool IsInMap(const std::array<float, 2> & xy){
    if (xy.at(0) < this->minx_) return false;
    if (this->maxx_ < xy.at(0)) return false;
    if (xy.at(1) < this->miny_) return false;
    if (this->maxy_ < xy.at(1)) return false;
    return true;
  }


  float minx_ = std::numeric_limits<float>::max();
  float maxx_ = std::numeric_limits<float>::min();
  float miny_ = std::numeric_limits<float>::max();
  float maxy_ = std::numeric_limits<float>::min();
  float resolution_ = 1; //地图分辨率,单位米

};

}//namespace plz

#endif //PILLAR_LOCALIZATION_GRID_MAP_H
