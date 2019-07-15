//
// Created by bohuan on 19-5-17.
//

#ifndef PILLAR_LOCALIZATION_GRID_MAP2D_H
#define PILLAR_LOCALIZATION_GRID_MAP2D_H


#include <pcl/kdtree/kdtree_flann.h>
#include "grid_map.h"
#include "registration.h"

typedef pcl::KdTreeFLANN<pcl::PointXY, ::flann::L1<float>> Manhattan2DKdTree_T;//貌似不对
typedef pcl::KdTreeFLANN<pcl::PointXY, ::flann::L2_Simple<float>> Euclidean2DKdTree_T;
//typedef pcl::KdTreeFLANN<pcl::PointXY> Manhattan2DKdTree_T;
typedef pcl::KdTreeFLANN<pcl::PointXYZ> KdTree_T;

#ifdef PLZ_USE_2DL2
typedef Euclidean2DKdTree_T KdTree2D;
#else
typedef Manhattan2DKdTree_T KdTree2D;
#endif

namespace plz {

class Grid_Map2d : public Grid_Map<PointCloud2D_T> {
 public:
  typedef std::shared_ptr<Grid_Map2d> Ptr;
  virtual void SetGlobalMap(const PointCloud2D_T::Ptr &global_map_ptr, const float &resolution) override;

  void GetPoints(const PointCloud2D_T &sample, PointCloud2D_T::Ptr result_ptr){
    if (sample.size() == 0) {
      std::cerr << "the sample size is 0" << std::endl;
      exit(-1);
    }

    if (result_ptr == nullptr) {
      std::cerr << "resutl is null ptr" << std::endl;
      exit(-1);
    }
    result_ptr->clear();
    for (auto &it : is_visited_) {
      for (auto &arg : it) {
        arg = false;
      }
    }
    //debug begin
   // FILE * fout = fopen("a.txt","w");
   // int sb =0;

    //debug end
    //std::cout<<sample.size() << std::endl;
    for (const auto &it : sample) {
      //fprintf(fout, "%lf %lf\n", it.x, it.y);
      //fprintf(fout, "%lf %lf %lf %lf\n", this->minx_,
      //        this->maxx_,
      //        this->miny_,
      //        this->maxy_);
      //if (it.x < this->minx_ || this->maxx_ < it.x) {
        //fprintf(fout, "xx:%lf %lf\n", it.x, it.y);
      //}
      //if (it.y < this->miny_ || this->maxy_ < it.y){
        //fprintf(fout, "yy:%lf %lf\n", it.x, it.y);
      //}
      //std::cout<<"@@"<<it.x<<" "<<it.y << std::endl;
      if (it.x < this->minx_ || this->maxx_ < it.x) continue;
      if (it.y < this->miny_ || this->maxy_ < it.y) continue;
      if (std::isnan(it.x))   continue;
      //++sb;
      //std::cout<<"ffffffffff"<<it.x<<" "<<it.y<<std::endl;
      auto xy = Convert2XY(it);
      //std::cout<<xy.at(0)<<" "<<xy.at(1)<<" "<<it.x<<" "<<it.y<<"]]]" << std::endl;

      if (true == is_visited_.at(xy.at(0)).at(xy.at(1))){
        continue;
      }
      for (const auto &arg : *grid_map2d_.at(xy.at(0)).at(xy.at(1)) ) {
        if (this->IsInMap({arg.x, arg.y})) {
          result_ptr->push_back(arg);
        }
      }
      is_visited_.at(xy.at(0)).at(xy.at(1)) = true;
    }

  }


 protected:

  std::array<int32_t, 2> Convert2XY(const pcl::PointXY &xy) {
    int32_t x = static_cast<int32_t>((xy.x - this->minx_)
        / (this->maxx_- this->minx_)
            /// this->resolution_
        * static_cast<float>(grid_map2d_.size() - 1));
    int32_t y = static_cast<int32_t>((xy.y - this->miny_)
        / (this->maxy_- this->miny_)
            /// this->resolution_
        * static_cast<float>(grid_map2d_[0].size() - 1));
    return {x, y};
  };

  std::vector<std::vector<int8_t> > is_visited_;
  //std::vector<std::vector<KdTree2D>> grid_kdtree_;
  std::vector<std::vector<PointCloud2D_T::Ptr>> grid_map2d_;

};

} //namespace

#endif //PILLAR_LOCALIZATION_GRID_MAP2D_H
