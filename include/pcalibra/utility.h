/**
 * @pcl-fit-line.h
 * @brief 主要实现点云拟合一条直线的工作。
 * @author bohuan
 * @email bohuan.xue@gmail.com
 * @version 0.0.1
 * @date 2019-06-27
 * */

#ifndef PCALIBRA_UTILITY_H
#define PCALIBRA_UTILITY_H

#include <opencv2/opencv.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/core/types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include "pcalibra/pcl-pcalibra.h"
#if (CV_VERSION_MAJOR != 4)
//#error "The OpenCV version is not larger than 4! CHECK your pcl version(it dependented on opencv)"
#endif

#define DEBUG_OUT(x) std::cerr<<__FILE__<<"have error in " <<__FUNCTION__ << "and line is " <<__LINE__ << std::endl;\
std::cerr<<(x)<<std::endl;\
exit(-1);


#define FEPS 1e-6  ///< 2个float误差小于这个数值，则视为相同

#define FEQUAL(a, b) ((fabs(a)-fabs(b)) < FEPS)  ///<判断2个浮点数是否相同


namespace pcalibra {
typedef cv::Point3d Point3;
typedef cv::Point2d Point2;


//#define EPoint EVector
typedef Eigen::Matrix<double, 3, 1> EVector;
typedef EVector EPoint;

typedef class EPlane{
 public:
  EPlane():coefficients({NAN,NAN,NAN,NAN}){}
//  EPlane(const std::array<double, 4> &arg):coefficients(arg){}
  EPlane(const Eigen::Matrix<double, 4, 1> &arg):coefficients(arg){}
  Eigen::Matrix<double, 4, 1> coefficients;
}EPlane;

class ELine{
 public:
  EPoint start;
  EPoint end;
  EPoint vector;
  void AdjustFromStartEnd(){
    //TODO
    exit(-1);
  }
  void AdjustFromStartVector();

};



/**
 * @brief 对Point3额外添加了intensity的信息,
 * 使得保留intensity的情况下，还可以使用使用原有的3D函数*/
typedef class Point3I : public Point3 {
 public:
  using Point3::x;
  using Point3::y;
  using Point3::z;
  double intensity;

  template<typename _Tp>
  Point3I(_Tp _x, _Tp _y, _Tp _z, double _intensity);

} Point3I;


/**
 * @brief 线的描述
 * @bug 没考虑线段在xy平面上的情况，如果出现这个情况则直接崩溃
 * @todo 修复上述bug, 实现AdjustFromStartEnd函数*/
typedef class Line3 {
 public:
  Point3 start; ///< 起点在xy平面上的坐标
  Point3 end;  ///< 从start开始，沿着y轴开始测度为1的坐标,如果线段在xy平面上
  Point3 vector; ///< 方向向量
  void AdjustFromStartEnd(); ///< TODO 给出2点坐标，归化到这个类的坐标上

  /**
   * @brief 给出一个点和向量，归化坐标*/
  void AdjustFromVector();

} Line3;


template<typename Point3_>
class PointCloud3D_ {
 public:
  PointCloud3D_<Point3_>();
  PointCloud3D_(const PointCloud3D_<Point3_> &pointcloud);


  template <typename _Tp>
  PointCloud3D_(const PointCloud3D_<_Tp> &pointcloud);


  /**
   * @brief 好像写垃圾了的一个函数
   * 没啥用*/
  template<typename _Tp>
  operator PointCloud3D_<_Tp>() ;


  PointCloud3D_& operator = (const PointCloud3D_& t);


  std::vector<Point3_> points;
  size_t size(){
    return points.size();
  }
} ;

typedef PointCloud3D_<Point3I> PointCloud3DI;
typedef PointCloud3D_<Point3> PointCloud3D;

/**
 * @brief 用来读取点云用的*/
typedef class PointCloudLoader{
 public:
  /**
   * @brief 根据path读取点云
   * @note 点云的格式应该如下：
   * x1 y1 z1 intensity1
   * x2 y2 z2 intensity2
   * x3 y3 z3 intensity3
   * ..................
   * ..................
   * ..................
   * xn yn zn intensityn
   * @param path 文件路径*/
  void Load(const std::string &path);
  void LoadVelodyne(const std::string &path);

  void GetPointCloud(std::vector<pcl::PointCloud<pcl::PointXYZI>> *ret){
    *ret = pointcloud_;
  }



 private:
  std::vector<pcl::PointCloud<pcl::PointXYZI> > pointcloud_;
  size_t size(){
    return pointcloud_.size();
  }
} PointCloudLoader;


/**
 * @brief 输出点云(pcalibra格式)到文件，通常用来可视化,会忽略掉intensity信息
 * @param cloud 输入的点云
 * @param path 输出的文件路径*/
template<typename _Tp>
void OutputPoints(const PointCloud3D_<_Tp> &cloud, const std::string &path);

/**
 * @brief 输出点云(pcl格式)到文件，通常用来可视化,会忽略掉intensity信息
 * @param cloud 输入的点云
 * @param path 输出的文件路径*/
template<typename _Tp>
void OutputPoints(const pcl::PointCloud<_Tp> &cloud, const std::string &path);

typedef class Plane{
 public:
  Plane():coefficients({NAN,NAN,NAN,NAN}){}
  Plane(const std::array<double, 4> &arg):coefficients(arg){}
  std::array<double, 4> coefficients;

  double & a(){ return coefficients.at(0);}

  double const & a()const{ return coefficients.at(0);}

  double & b(){ return coefficients.at(1);}

  double const & b()const{ return coefficients.at(1);}

  double & c(){ return coefficients.at(2);}

  double const & c()const{ return coefficients.at(2);}

  double & d(){ return coefficients.at(3);}

  double const & d()const{ return coefficients.at(3);}

} Plane;


}//namespace


namespace pcalibra{

template<typename _Tp> inline
Point3I::Point3I(_Tp _x, _Tp _y, _Tp _z, double _intensity):Point3_(_x,_y,_z),intensity(_intensity){}


template <typename Point3_> inline
PointCloud3D_<Point3_>::PointCloud3D_() {
  this->points.clear();
}

template<typename Point3_> inline
PointCloud3D_<Point3_>::PointCloud3D_(const pcalibra::PointCloud3D_<Point3_> &pointcloud) {
  this->points = pointcloud.points;
  //this->size = pointcloud.size;
}

template <typename Point3_> inline
PointCloud3D_<Point3_>& PointCloud3D_<Point3_>::operator=(const pcalibra::PointCloud3D_<Point3_> &t) {
  this->points = t.points;
  //this->size = t.size;
}


template<typename Point3_> template<typename _Tp> inline
PointCloud3D_<Point3_>::operator PointCloud3D_<_Tp>(){
  PointCloud3D_<_Tp> ret;
  ret.points.clear();
  for (auto it : this->points) {
    _Tp tp;
    tp.x = it.x;
    tp.y = it.y;
    tp.z = it.z;
    ret.points.push_back(it);
  }
  return ret;
}

template <typename Point3_> template <typename _Tp> inline
PointCloud3D_<Point3_>::PointCloud3D_(const PointCloud3D_<_Tp> &pointcloud) {
  this->points.clear();
  for (auto it : pointcloud.points) {
    this->points.push_back({it.x, it.y, it.z});
  }
}



template<typename _Tp>
void OutputPoints(const PointCloud3D_<_Tp> &cloud, const std::string &path){
  FILE *fout = fopen(path.c_str(), "w");
  for (auto it : cloud.points) {
    fprintf(fout, "%lf %lf %lf\n", it.x, it.y, it.z);
  }
  fclose(fout);
}


template<typename _Tp>
void OutputPoints(const pcl::PointCloud<_Tp> &cloud, const std::string &path){
  FILE *fout = fopen(path.c_str(), "w");
  for (auto it : cloud) {
    fprintf(fout, "%lf %lf %lf\n", it.x, it.y, it.z);
  }
  fclose(fout);

}



} //namespace

#endif //PCALIBRA_UTILITY_H
