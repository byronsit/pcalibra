//
// Created by bohuan on 19-6-28.
//

#include "pcalibra/ground-extraction.h"
#include "pcalibra/pcl-pcalibra.h"

#include <numeric>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>


/*保存一些常亮*/
namespace {
double NORMAL_RADIUS = 0.1; ///<normal半径搜索的半径


}


namespace pcalibra {
/**
 * @brief 直接粗糙的提取地面
 * 把高度低于0的点全部提出来(假设雷达高于地面)
 * 随后求这些点z信息的方差，把高度大于均值+方差的所有点再次滤除，
 * 剩下的就是粗糙的地面信息
 * @param pointcloud 输入的原始点云(raw data,推荐去除过车体或其他简单杂点的的非raw data)
 * @param pclpc3 输出粗糙的的点云地面*/
void ExtractRawGround(const pcalibra::PointCloud3D &pointcloud, pcl::pclpc3 *pclpc3) {
  std::vector<double> z;
  for (auto it : pointcloud.points) {
    if (it.z >= 0) continue;
    z.push_back(it.z);
  }
  std::sort(z.begin(), z.end());
  double sum = std::accumulate(z.begin(), z.end(), 0);
  double avg = sum / z.size();
  sum = 0;
  for (auto it : z){
    sum += (it - avg)*(it - avg);
  }
  double var = sum / (z.size()-1);
  std::cout<<"height avg:"<<avg<<" var:"<<var<<std::endl;
  //pcl::pclpc3::Ptr kdtree_input(new pcl::pclpc3);

  for (auto it : pointcloud.points) {
    if (it.z >= avg + var/2)  continue;
    pcl::PointXYZ xyz;
    Pcalibra2PCL(it, &xyz);
    //kdtree_input->push_back(xyz);
    pclpc3->push_back(xyz);
  }

}

/**
 * @brief 去除部分高的地面
 * 任何一个点，如果周围半径为@arg1 范围内有点比它矮@arg0 个测度，则不可能是地面点
 * @param arg0 在arg1范围内高度差为arg0的点，则丢弃
 * @param arg1 所搜半径
 * */
void RemoveHighGround(const pcl::pclpc3 &pclpc3, pcl::pclpc3 *output, double arg0=0.05, double arg1=0.1){
  output->points.clear();
  pcl::Euclidean2DKdTree_T kdtree;
  pcl::pclpc2::Ptr tmp(new pcl::pclpc2);
  for (auto it : pclpc3.points) {
    pcl::PointXY xy;
    xy.x = it.x;
    xy.y = it.y;
    tmp->push_back(xy);
  }

//  std::cout<<pclpc3.points.size() << std::endl;
//  std::cout<<tmp->points.size() << std::endl;
  kdtree.setInputCloud(tmp);
  std::vector<int> indeces;
  std::vector<float> dis;
  for (auto it : pclpc3.points) {
    bool flag(true);
    pcl::PointXY xy;
    xy.x = it.x;
    xy.y = it.y;

    kdtree.radiusSearchT(xy, arg1, indeces, dis);
    for (auto index : indeces) {
      if (fabs(pclpc3.points.at(index).z - it.z) > arg0) {
        flag = false;
        break;
      }
    }
    if (flag) {
      output->points.push_back(it);
    }
  }
}

/**
 * @brief 给定点云，计算点云的normal
 * 注意输入输出的点云类型不完全一样。输出带了intensity(没用)和normal
 * @param pointcloud 输入的点云
 * @param output 输出的点云*/
void CalPointsNormal(const pcl::pclpc3 &pointcloud, pcl::pclpc3in *output){
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::pclpc3::Ptr tmp(new pcl::pclpc3(pointcloud));
  ne.setInputCloud(tmp);
  ne.setRadiusSearch(NORMAL_RADIUS);
  pcl::pclpcn pcnormal;
  ne.compute(pcnormal);
  pcl::pclpc3in t;
  pcl::concatenateFields(*tmp, pcnormal, t);
  std::vector<int> indeces;

  for (auto &it : t.points) { //让向量都向着天
    if (isnan(it.normal_z)) {
      it.x = it.y = it.z = NAN;
    }
  }
  pcl::removeNaNFromPointCloud(t, *output, indeces);//去除nan
  for (auto &it : output->points) { //让向量都向着天
    //if(it.data_n[0] != it.normal_x){
    //  for (int i = 0; i < 3; ++ i) std::cout<<it.data_n[i]<<" "; std::cout<<std::endl;
    //  std::cout<<it.normal_x <<" "<<it.normal_y<<" "<<it.normal_z << std::endl;
    //}
    assert(it.data_n[0] == it.normal_x);
    if (it.normal_z < 0) {
      for (int i = 0; i < 3; ++ i) {
        it.data_n[i] = - it.data_n[i];
      }
    }
  }
}

template<typename _Tp>
pcl::Normal operator + (const pcl::Normal &a, const _Tp &b){
  pcl::Normal ret;
  /*
  ret.normal_x = a.normal_x + b.normal_x;
  ret.normal_y = a.normal_y + b.normal_y;
  ret.normal_z = a.normal_z + b.normal_z;
  return ret;
   */

  for (int i =0; i < 3; ++ i) {
    ret.normal[i] = a.normal[i] + b.normal[i];
  }
  return ret;
}

/**
 * @brief 求2个向量之间的夹角
 * @param a 第一个normal
 * @param b 第二个normal*/
template<typename _Tp>
double operator ^ (const pcl::Normal &a, const _Tp &b){
  double ret(0);
  for (int i =0; i < 3; ++ i) {
    ret += a.normal[i] * b.normal[i];
  }
  /*
  if (isnan(acos(ret))){
    std::cout<<ret<<std::endl;
    for (int i =0; i < 3; ++ i) {
     std::cout<<a.normal[i] <<" " <<b.normal[i]<<std::endl;
    }
    std::cout<<b.normal_x<<std::endl;
    exit(-1);
  }
   */
  return acos(ret);
}


/**
 * @brief 根据带normal的点云，提取最真实的地面区域
 * 因为不能保证雷达平放，所以要求出normal的众数.
 * @param pointcloud 输入带normal的点云
 * @param output 输出普通点云，带地面信息
 * @bug 没考虑全部是nan的情况
 * @bug 没考虑input的点是0个的情况
 * @bug 没考虑没有点的情况*/

void ExtractFinalGround(const pcl::pclpc3in &pointcloud, pcl::pclpc3 *output){
  output->clear();
  pcl::Normal normal;
  memset(normal.data_n, 0, sizeof(normal.data_n));
  //normal.normal_x = normal.normal_y = normal.normal_z = 0;
  int tot_nan(0);
  for (auto it : pointcloud) {
    if (isnan(it.normal_z) == true) {
      ++tot_nan;
      continue;
    }
    assert (it.normal_z >= 0);
    normal = normal + it;
  }
  for (int i = 0; i < 3; ++ i) {
    normal.data_n[i] /= pointcloud.size() - tot_nan;
  }
  double sum(0);
  std::vector<double>t;
  for (auto it : pointcloud) {
    if (isnan(it.normal_z)) continue;
    t.push_back(normal ^ it);
    sum += normal ^ it;
  }
  std::sort(t.begin(), t.end());
  double avg = sum / (pointcloud.size() - tot_nan); //平均偏差
  sum = 0;
  for (auto it : pointcloud) {
    if (isnan(it.normal_z)) continue;
    sum += (normal ^ it) * (normal ^ it);
  }
  std::cout<<"normal:"<<normal.normal_x<<" "<<normal.normal_y<<" "<<normal.normal_z<<std::endl;
  std::cout<< tot_nan << std::endl;
  double var = sum / (pointcloud.size() - tot_nan - 1);
  std::cout<<"avg:"<< avg <<" var:"<<var<<" number:"<<(pointcloud.size() - tot_nan - 1)<< std::endl;
  for (auto it : pointcloud) {
    if (isnan(it.normal_z)) continue;
    if ((normal ^ it) < avg) {
      pcl::PointXYZ xyz;
      memmove(xyz.data, it.data, 3 * sizeof(float));
      output->push_back(xyz);
    }
  }
}




}// namespace




namespace pcalibra{




void GroundExtraction::Extract(const pcalibra::PointCloud3D &pointcloud, PointCloud3D *output) {
  pcl::pclpc3::Ptr raw_ground_ptr(new pcl::pclpc3);
  pcl::pclpc3in::Ptr normal_ground_ptr(new pcl::pclpc3in);
  pcl::pclpc3::Ptr output_ptr(new pcl::pclpc3);
  pcl::pclpc3::Ptr low_ground_ptr(new pcl::pclpc3);
  ExtractRawGround(pointcloud, raw_ground_ptr.get());
  OutputPoints(*raw_ground_ptr, "raw_ground.txt");
  RemoveHighGround(*raw_ground_ptr, low_ground_ptr.get());
  OutputPoints(*low_ground_ptr, "low_ground.txt");
  CalPointsNormal(*low_ground_ptr, normal_ground_ptr.get());
  OutputPoints(*normal_ground_ptr, "normal_ground.txt");
  ExtractFinalGround(*normal_ground_ptr, output_ptr.get());
  OutputPoints(*output_ptr, "final_ground.txt");
}

}//namespace
