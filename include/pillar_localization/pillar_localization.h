//
// Created by bohuan on 19-5-17.
//

#ifndef PILLAR_LOCALIZATION_PILLAR_LOCALIZATION_H
#define PILLAR_LOCALIZATION_PILLAR_LOCALIZATION_H

#include "registration2d.h"
#include "grid_map2d.h"
#include <pcl/io/pcd_io.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <velodyne_pointcloud/pointcloudXYZIR.h>
#include <velodyne_pointcloud/point_types.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/organized.h>
#include <pcl/search/impl/organized.hpp>




#define PLZ_DIS2_2(a, b) (((a).x-(b).x)*((a).x-(b).x) + ((a).y-(b).y)*((a).y-(b).y))
#define PLZ_DIS3_2(a, b) (((a).x-(b).x)*((a).x-(b).x) + ((a).y-(b).y)*((a).y-(b).y) + ((a).z-(b).z)*((a).z-(b).z))

namespace plz {


class PillarLocalization {
 public:

  //检查这个下标
  //points是传来的点云，index是弧度位置，point是指定的点。 这样的那个点是符合intensity标准的
  bool checkPoints(const std::vector<std::vector<pcl::PointXYZI>> &points, size_t index, const pcl::PointXYZI &point, pcl::PointXY* output) {
    //std::cout<<"check " << point.x<<" "<<point.y << std::endl;
    size_t length = points.at(0).size();
    length = 3000; //
    std::vector<PointCloud_T> pc;
    std::vector<pcl::PointXYZ> centroid;
    pc.resize(points.size());
    centroid.resize(points.size());
   for (int i = 0; i < points.size(); ++ i) {
      for (int j = index; j != (index - 1 + length) % length ;  j = (j + 1) % length) {
        auto p = points.at(i).at(j);

        if (std::isnan(p.x)) continue; //去除nan

        if (p.intensity < this->min_intensity_)  continue; //去除低反射率
       // if (-0.663062<=point.x&&point.x<=-0.663060) {
       //   std::cout<<p.x<<" "<<p.y<<" "<<p.z<<" "<<p.intensity<<" "<<PLZ_DIS2_2(p, point)<<std::endl;
       // }

        if (PLZ_DIS2_2(p, point) > distance_ * distance_) break; //点距离太远也不行。甚至可以break

      //  if (-0.663062<=point.x&&point.x<=-0.663060) {
      //    std::cout<<"@@@"<<p.x<<" "<<p.y<<std::endl;
      //  }


        //std::cout<<p.x<<" "<<p.y<<" "<<p.z<<" "<<p.intensity<<std::endl;
        pcl::PointXYZ xyz;
        xyz.x = p.x;
        xyz.y = p.y;
        xyz.z = p.z;
        pc.at(i).push_back(xyz);
        //std::cout<<"@@"<<xyz.x<<" "<<xyz.y<<" "<<xyz.z<<" "<<p.intensity<<std::endl;
      }

     Eigen::Vector4f cen;
     if (pc.at(i).size() < arg1_) {
       centroid.at(i).x = NAN;
       centroid.at(i).y = NAN;
       centroid.at(i).z = NAN;
     }
     else {
       pcl::compute3DCentroid(pc.at(i), cen);
       centroid.at(i).x = cen[0];
       centroid.at(i).y = cen[1];
       centroid.at(i).z = cen[2];
       //所有点不能距离质心太远
       for (auto it : pc.at(i)) {
         if (PLZ_DIS2_2(centroid.at(i), it) > distance_ * distance_ / 2) {
           centroid.at(i).x = NAN;
           centroid.at(i).y = NAN;
           centroid.at(i).z = NAN;
         }
       }
     }
   }

    /*-----   判定哪一个才是真正的质心   ------*/
    /* 在distance范围内若干个质心，求质心      */
    float totx(0), toty(0);
    int cnt(0);
    for (int i = 0; i < points.size(); ++ i) {
      if (std::isnan(centroid.at(i).x)) continue;
      ++cnt;
      totx += centroid.at(i).x;
      toty += centroid.at(i).y;
    }
    if (cnt >= arg0_) {
      totx /= cnt;
      toty /= cnt;
    }
    else {
      cnt = 0;
    }

    if (cnt == 0) {
      return false;
    }
    else
    {
      output->x = totx;
      output->y = toty;
      return true;
    }
  }

  void CalKeyPoints(const std::vector<pcl::PointCloud<pcl::PointXYZI>> &points, PointCloud2D_T::Ptr output_ptr){
    size_t height = points.size();
    std::vector<std::vector<pcl::PointXYZI>> pc;
    pc.resize(height);
    size_t max_size=0;
    for (auto it : points) {
      max_size = std::max(max_size, it.size());
    }
    max_size = 3000;
    for (auto &it : pc) {
      it.resize(max_size);
      for (auto &ity : it) {
        ity.x = ity.y = ity.z = NAN;
      }
    }

    //重新按照角度映射所有的点
    for (int i = 0; i < height; ++ i) {
      for (auto it : points.at(i)) {
        float_t div = (atan2(it.y, it.x) + M_PI) / (M_PI * 2);
        size_t index =  ceil((max_size - 1) * div);
        pc.at(i).at(index) = it;
      }
    }

    std::vector<pcl::PointXYZI> visited;//记录被选定过的标识点
    auto is_visited = [&](pcl::PointXYZI p)->bool{
      for (auto it : visited) {
        if (PLZ_DIS2_2(it, p) < distance_ * distance_ * 1.9383) {
          return true;
        }
      }
      return false;
    };

    for (size_t i = 0; i < max_size; ++ i) { //点
      for (size_t j = 0; j < height; ++ j) { //线
        auto p = pc.at(j).at(i);
        if (p.intensity < this->min_intensity_) continue; //intensity太小，抛掉
        if (is_visited(p)) continue;          //已经被搜索过的，continue
        visited.push_back(p);
        //std::cout<<p.x<<" |||||| "<<p.y<<std::endl;
        pcl::PointXY xy;
        if (checkPoints(pc, i, p, &xy) == true) {
          // std::cout<<"hahaha "<<xy.x<<" "<<xy.y<< std::endl;
          output_ptr->push_back(xy);
        }
        //exit(-1);
      }
    }
    //for debug
    //std::cout<<output_ptr->size()<< std::endl;
    //for (auto it : *output_ptr) {
    // std::cout<<it.x <<" "<<it.y << std::endl;
    //}
  }



  void VelodyneFilter3dto2d(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR> & input, PointCloud2D_T::Ptr output_ptr) {
    PointCloud_T::Ptr points_ptr(new PointCloud_T);
    points_ptr->clear();
    std::vector<pcl::PointCloud<pcl::PointXYZI>> points;
    points.resize(4);
    for (auto it : input) {
      pcl::PointXYZI xyzi;
      xyzi.x = it.x;
      xyzi.y = it.y;
      xyzi.z = it.z;
      xyzi.intensity = it.intensity;
      if (it.ring >= points.size()) {
        points.resize(it.ring + 1);
      }
      points.at(it.ring).push_back(xyzi);
    }
    CalKeyPoints(points, output_ptr);
    if (output_ptr->size() == 0) {
      return;
    }

    //pub出选中的点
   // for (auto it : *output_ptr) {
   //   std::cout <<it.x <<" "<<it.y << std::endl;
   // }
    PubPointCloud(output_ptr); //DEBUG
    //exit(-1);
  }


  void LoadGlobalMapXYZ(const std::string &path) {
    global_map_ptr_.reset(new PointCloud2D_T);
    PointCloud_T global_map;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, global_map) == -1) {
      //TODO DEBUG ERROR
      exit(-1);
    }
    for (const auto &it : global_map) {
      pcl::PointXY xy;
      xy.x = it.x;
      xy.y = it.y;
      global_map_ptr_->push_back(xy);
    }
  }

  void LoadGlobalMapXY(const std::string &path) {
    global_map_ptr_.reset(new PointCloud2D_T);
    if (pcl::io::loadPCDFile<pcl::PointXY>(path, *global_map_ptr_) == -1) {
      //TODO DEBUG ERROR
      exit(-1);
    }
  }

  void Initialzing(const float &resolution) {
    grid_map2d_ptr_.reset(new Grid_Map2d);
    grid_map2d_ptr_->SetGlobalMap(global_map_ptr_, resolution);
  }

  void WorkVelodyne(pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr input,  //当前帧点云
                    const Eigen::Matrix4f &guess_odom,            //当前帧所在的odom,guess的结果
                    PointCloud2D_T::Ptr sample_ptr,               //样本点云，在global map的位置
                    Eigen::Matrix4f *result_ptr){                 //输出的结果
    //pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr temp(new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
    PointCloud_T::Ptr temp(new PointCloud_T), temp2(new PointCloud_T);
    PointCloud2D_T::Ptr source_ptr(new PointCloud2D_T);

#ifdef DEBUG
    VelodyneFilter3dto2d(*input, source_ptr);
#else
    VelodyneFilter3dto2d(*input, source_ptr);


    Points2dto3d(*source_ptr, temp);
    pcl::transformPointCloud(*temp, *temp2, guess_odom);
    Points3dto2d(*temp2, source_ptr);
    //std::cout<<"size: "<< source_ptr->size() << std::endl;
    if (source_ptr->size() <= 3)  {
      std::cerr<<"source points too less" << std::endl;
      return;
    }
#endif
    if (sample_ptr == nullptr) {
      Work(source_ptr, source_ptr, result_ptr);
    }
    else {
      Work(source_ptr, sample_ptr, result_ptr);
    }
  }

  //普通运算
  void WorkCommon(pcl::PointCloud<pcl::PointXYZI>::Ptr input,
                  const Eigen::Matrix4f &guess_odom,            //当前帧所在的odom,guess的结果
                  PointCloud2D_T::Ptr sample_ptr,               //样本点云，在global map的位置
                  Eigen::Matrix4f *result_ptr){                 //输出的结果
    Euclidean2DKdTree_T kdtree;
    PointCloud_T::Ptr points(new PointCloud_T);
    for (auto it : input->points) {
      if (std::isnan(it.x)) {
        continue;
      }
      if (it.intensity < this->min_intensity_) {
        continue;
      }
      pcl::PointXYZ xyz;
      xyz.x = it.x;
      xyz.y = it.y;
      xyz.z = 0;
      points->push_back(xyz);
    }
    //kdtree.setInputCloud(points);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster;
    std::vector<pcl::PointIndices> indices;
    cluster.setInputCloud(points);
    cluster.setMinClusterSize(arg0_);
    //cluster.setSearchMethod(kdtree);
    cluster.setClusterTolerance(distance_ * distance_);
    cluster.extract(indices);
    for (int i = 0; i < indices.size(); ++ i) {
      std::cout << indices.at(i).indices.size() << std::endl;
    }




  }

  void Work(PointCloud2D_T::Ptr source_ptr, PointCloud2D_T::Ptr sample_ptr, Eigen::Matrix4f *result_ptr) {
    if (source_ptr == nullptr)  {
      std::cout<<"source_ptr == nullptr" << std::endl;
      exit(-1);
    }
    if (sample_ptr == nullptr) {
      std::cout<<"sample_ptr == nullptr" << std::endl;
      exit(-1);
    }
    static PointCloud2D_T::Ptr target_ptr(new PointCloud2D_T);
    static PointCloud2D_T::Ptr output_ptr(new PointCloud2D_T);
    grid_map2d_ptr_->GetPoints(*sample_ptr, target_ptr);
    //std::cout<<target_ptr->size() << std::endl;
    //PubPointCloud(target_ptr);
    registeration_.SetSourcePoints(source_ptr);
    registeration_.SetTargetPoints(target_ptr);
    registeration_.ComputeTransformation(output_ptr, result_ptr);
    /*
    for (auto it : *source_ptr) {
      std::cout<<"sensor: " << it.x <<" " << it.y<< std::endl;
    }

    for (auto it : *target_ptr) {
      std::cout<<"in map: " << it.x <<" " << it.y<< std::endl;
    }
    std::cout<<std::endl;
     */
  }

  void SetPublisher(ros::Publisher *& pub){
    this->puber = pub;
  }

  void PubPointCloud(PointCloud2D_T::Ptr points){
    PointCloud_T::Ptr points3d(new PointCloud_T);
    Points2dto3d(*points, points3d);
    sensor_msgs::PointCloud2 ros_pc2;
    pcl::toROSMsg (*points3d, ros_pc2);
    ros_pc2.header.frame_id="velodyne";
    puber->publish(ros_pc2);
  }

  void SetDistance(const float &distance) {
    this->distance_ = distance;
  }

  void SetMinIntensity(const int32_t intensity) {
    this->min_intensity_ = intensity;
  }


 private:
  PointCloud2D_T::Ptr global_map_ptr_;
  Grid_Map2d::Ptr grid_map2d_ptr_;
  Registration2d registeration_;
  ros::Publisher *puber;
  float distance_ = 0.2; //2个点的距离差多少，可以视为在一个类里。 默认是0.1米
  int32_t min_intensity_ = 100;
  int32_t arg0_ = 3; //参数0,表示多少根线都扫到一块区域，就可以判定为是合法的
  int32_t arg1_ = 1; //参数1,表示在一根线上，多少个点扫到一块区域，可以判定为合法。

};
}

#endif //PILLAR_LOCALIZATION_PILLAR_LOCALIZATION_H
