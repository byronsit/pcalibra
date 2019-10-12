/**
 * @core.h
 * @brief 主类，整个程序的入口
 * @author bohuan
 * @email bohuan.xue@gmail.com
 * @version 0.0.1
 * @date 2019-07-03
 * */


#ifndef PCALIBRA_CORE_H
#define PCALIBRA_CORE_H

#include "pcalibra/pcl-pcalibra.h"
#include "pcalibra/utility.h"
#include "pcalibra/correspondence.h"
#include "pcalibra/pillar-extraction.h"
#include "pcalibra/optimization.h"


#include <pcl/registration/correspondence_estimation.h>
#include <pcl/filters/filter.h>


#include <pcl/registration/ndt.h>

namespace pcalibra {

class Core {
 public:
  Core(){
    isometry = Eigen::Isometry3d::Identity();
  }

  void SetReferencePointCloud(const pcl::pclpc3i &reference){
    reference_ = new pcl::pclpc3i(reference);
  }

  void SetAlignedPointCloud(const pcl::pclpc3i &aligned) {
    aligned_ = new pcl::pclpc3i(aligned);
  }


  /**
   * @brief 设置correspondence
   * 实际上就是穷举的方法，
   * 配的结果来进行检验就完事了，非常简单，用于初值不好的情况*/
  void SetCorrepondencePointLine(const size_t ref_number, const size_t alg_number){
      ELine l = lines_.at(ref_number); //line
      for (auto point : pcs_.at(alg_number)){
        EPoint p;
        p.x() = point.x;
        p.y() = point.y;
        p.z() = point.z;
        cor_.push_back(Correspondence(p ,l));
      }
  }

  /**
   * @brief 计算correspondence
   * 主要利用pcl的方式，这里简单的填充一些东西
   * 只考虑point和line的对应情况,
   * 但是最好只用在初值很好的情况下*/
  void GetCorrespondencePointLine(){
    //计算correspondence
    pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ, pcl::PointXYZ> est;
    pcl::pclpc3::Ptr line_points(new pcl::pclpc3);
    pcl::pclpc3::Ptr cen_points(new pcl::pclpc3);
    for (auto it: lines_){
      pcl::PointXYZ xyz;
      xyz.x = it.start.x();
      xyz.y = it.start.y();
      xyz.z = it.start.z();
      line_points->push_back(xyz);
    }
    for (auto it : pcs_){
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(it, centroid);
      pcl::PointXYZ xyz;
      xyz.x = centroid.x();
      xyz.y = centroid.y();
      xyz.z = centroid.z();
      cen_points->push_back(xyz);
    }
    est.setInputSource(cen_points);
    est.setInputTarget(line_points);
    pcl::Correspondences cores;
    est.determineReciprocalCorrespondences(cores);
    for (auto it : cores){
      ELine l = lines_.at(it.index_match); //line
      for (auto point : pcs_.at(it.index_query)){
        EPoint p;
        p.x() = point.x;
        p.y() = point.y;
        p.z() = point.z;
        cor_.push_back(Correspondence(p ,l));
      }
    }
  }

  /**
   * @brief 除了上面的correspondence的对应关系外，还要额外考虑地面的对应关系。(很烦啊!)*/
  void GetCorrespondenceAll(){
    //TODO
  }


  /**
   * @brief 2个点云利用杆子做初步对齐。*/
  void AlignLines(){
    PillarExtraction pe;
    CHECK_NE((u_int64_t)reference_, 0) << "have not set reference\n";
    CHECK_NE((u_int64_t)aligned_, 0) << "have not set aligned\n";

    pe.Extract(*reference_);

    pe.GetLines(&lines_);

    /*
    for (auto it : lines_){
      std::cout<<"line:\n";
      std::cout<<it.start<<"\n"<<it.vector<<'\n';
      std::cout<<"\n\n";
    }
    */

    pe.Extract(*aligned_);
    pe.GetPointCloud(&pcs_);
    GetCorrespondencePointLine(); //利用上述信息，配对好对应关系
    Optimization opt;
    opt.SetCorrespondence(cor_);
    opt.Solve();

    Eigen::Vector3d trans;
    Eigen::Quaterniond q;
    opt.GetResult(&trans,&q);
    std::cout<<trans<<"\n\n"<<q.toRotationMatrix()<<std::endl;
    std::cout<<"----end----"<<std::endl;
  }
  /**
   * @brief 对给定的点云和line，求一个初值
   */



  /*
  void foo(ELine ref0, ELine alg0, ELine ref1, ELine alg1){
    //已经所有的line的start都是z=0的平面上了
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d trans = Eigen::Vector3d::Zero();

    Eigen::Matrix<double, 3, 1> n = (ref0.vector + alg0.vector) / 2;
    Rodrigues(n, M_PI_2, rotation);

   // ref0.vector
  }
   */

  void CheckAndChoseResult(Eigen::Vector3d trans, Eigen::Quaterniond t) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(t);
    T.matrix().block(0,4,3,1) = trans;
    pcl::pclpc3 ref, alg;
    pcl::copyPointCloud(*reference_, ref);
    pcl::copyPointCloud(*aligned_, alg);
    pcl::transformPointCloud(alg, alg, T.matrix());
    float minx = 1e100, maxx=-1e100, miny=1e100, maxy=-1e100, minz=1e100, maxz=-1e100;
    for (auto it : ref) {
      minx = std::min(minx, it.x);
      miny = std::min(miny, it.y);
      minz = std::min(minz, it.z);
      maxx = std::max(maxx, it.x);
      maxy = std::max(maxy, it.y);
      maxz = std::max(maxz, it.z);
    }
    for (auto it : alg) {
      minx = std::min(minx, it.x);
      miny = std::min(miny, it.y);
      minz = std::min(minz, it.z);
      maxx = std::max(maxx, it.x);
      maxy = std::max(maxy, it.y);
      maxz = std::max(maxz, it.z);
    }
    //得到合理范围内的bounding box,只要匹配这里的东西就足够了

    for (auto &it : ref) {
      if (it.x < minx || maxx < it.x ) it.x = NAN;
      if (it.y < miny || maxy < it.y ) it.y = NAN;
      if (it.z < minz || maxz < it.z ) it.z = NAN;
    }
    for (auto &it : alg) {
      if (it.x < minx || maxx < it.x ) it.x = NAN;
      if (it.y < miny || maxy < it.y ) it.y = NAN;
      if (it.z < minz || maxz < it.z ) it.z = NAN;
    }
    std::vector<int> tmp;
    pcl::removeNaNFromPointCloud(ref,  ref, tmp);
    pcl::removeNaNFromPointCloud(alg,  alg, tmp);

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setResolution (0.5);   //设置NDT网格网格结构的分辨率（voxelgridcovariance）
    ndt.setInputSource(pcl::pclpc3::Ptr(new pcl::pclpc3(alg)));  //源点云
    ndt.setInputTarget(pcl::pclpc3::Ptr(new pcl::pclpc3(ref)));  //目标点云
    ndt.align(alg);

    Eigen::Matrix4f res = ndt.getFinalTransformation();
    auto a = res(0,0) - 1;
    auto b = res(1,1) - 1;
    auto c = res(2,2) - 1;
    auto d = res(3,3) - 1;
    if (a*a + b*b + c*c + d*d > 1e-3) std::cout<<"not this"<< std::endl;
    else std::cout<<"ans is this" << std::endl;
    std::cout << res << std::endl;
    std::cout<<"-----------------"<<std::endl;
  }

  /**
   * @brief 用来穷举优化结果，寻找合适的初始值
   * 数据被分为2组，a和b两个组。每个组分别又一个align和一个ref变量,分别来自alg的点云和ref点云
   * 这些变量的数值都是用数字表示。
   * @param groub_a_alg a组的align变量 ……*/
  void OptimizeVector(int group_a_ref, int group_a_alg, int group_b_ref, int group_b_alg) {
    //1 set对应关系
    //2 穷举vector
    const int af[] = {1, -1, 1, -1};
    const int bf[] = {-1, 1, 1, -1};
    Eigen::Vector3d trans;
    Eigen::Quaterniond q;
    SetCorrepondencePointLine(group_a_alg, group_a_ref);
    SetCorrepondencePointLine(group_b_alg, group_b_ref);
 //   opt.Solve();
    for (int i = 0; i < 2; ++i) {
      Optimization opt;
      std::vector<Correspondence> cor;
      af[i] * lines2_.at(group_a_alg).vector;
      cor.push_back(Correspondence(lines_.at(group_a_ref), lines2_.at(group_a_alg) ^ af[i]));
      cor.push_back(Correspondence(lines_.at(group_b_ref), lines2_.at(group_b_alg) ^ bf[i]));
      opt.SetCorrespondence(cor);
      opt.Solve();
      opt.GetResult(&trans,&q);
    }




  }

  /**
   * @brief 这个align是穷举所有的情况，最后择优输出*/
  void Align(){
    PillarExtraction pe;
    CHECK_NE((u_int64_t)reference_, 0) << "have not set reference\n";
    CHECK_NE((u_int64_t)aligned_, 0) << "have not set aligned\n";

    pe.Extract(*reference_);
    pe.GetLines(&lines_);
    std::vector<pcl::pclpc3> tmp_ref;
    pe.GetPointCloud(&tmp_ref);
    pe.Extract(*aligned_);
    pe.GetLines(&lines2_);
    pe.GetPointCloud(&pcs_);


    /**
     * @brief pc是点云，求第n个点云的质心*/
    auto f = [](const pcl::pclpc3 &pc, pcl::PointXYZ *ret){
      pcl::PointXYZ xyz;
      xyz.x = xyz.y = xyz.z = 0;
      for (auto it : pc){
        xyz.x += it.x;
        xyz.y += it.y;
        xyz.z += it.z;
      }
      xyz.x /= pc.size();
      xyz.y /= pc.size();
      xyz.z /= pc.size();
      *ret = xyz;
    };
    pcl::PointXYZ ref_cen[2], alg_cen[2];
    f(tmp_ref.at(0), &ref_cen[0]);
    f(tmp_ref.at(1), &ref_cen[1]);
    f(pcs_.at(0), &alg_cen[0]);
    f(pcs_.at(1), &alg_cen[1]);


    cor_.clear();
   // SetCorrepondencePointLine(0,1);
   // SetCorrepondencePointLine(1,0);

    SetCorrepondencePointLine(0,0);
    SetCorrepondencePointLine(1,1);

    Optimization opt;
    opt.SetCorrespondence(cor_);
    opt.Solve();

    Eigen::Vector3d trans;
    Eigen::Quaterniond q;
    opt.GetResult(&trans,&q);

    // 把xyz进行tf(算出来的),然后和ref_cen 比较，看看最近的结果是否比较远(特别远<距离大于1>则false)
    /*
     * //貌似这段代码没啥用
    auto test = [&q, &trans, &ref_cen](const pcl::PointXYZ &xyz)->bool{
      Eigen::Vector3d p;
      p.x() = xyz.x;
      p.y() = xyz.y;
      p.z() = xyz.z;
      p = q.toRotationMatrix() * p + trans;
      auto e1 = sqrt(pow(p.x()-ref_cen[0].x,2) + pow(p.y()-ref_cen[0].y,2) + pow(p.z()-ref_cen[0].z,2));
      auto e2 = sqrt(pow(p.x()-ref_cen[1].x,2) + pow(p.y()-ref_cen[1].y,2) + pow(p.z()-ref_cen[1].z,2));
      std::cout<<e1<<" "<<e2<< std::endl;
      return std::min(e1,e2)<1;
    };
    auto b1 = test(alg_cen[0]);
    auto b2 = test(alg_cen[1]);

    std::cout<<trans<<"\n\n"<<q.toRotationMatrix()<<std::endl;
    if (!b1 || !b2){
      std::cout<<"WARNING: the first match is error, so we do the another match." << std::endl;
      cor_.clear();
      SetCorrepondencePointLine(0,0);
      SetCorrepondencePointLine(1,1);

      Optimization opt;
      opt.SetCorrespondence(cor_);
      opt.Solve();
      opt.GetResult(&trans,&q);
      test(alg_cen[0]);
      test(alg_cen[1]);
    }
     */


    //需要一个合理的算法






    std::cout<<trans<<"\n\n"<<q.toRotationMatrix()<<std::endl;
    std::cout<<"----end----"<<std::endl;
  }





 private:
  std::vector<Correspondence> cor_;
  pcl::pclpc3i* reference_;
  pcl::pclpc3i* aligned_;
  std::vector<ELine> lines_;
  std::vector<ELine> lines2_;
  std::vector<pcl::pclpc3> pcs_;
  Eigen::Isometry3d isometry;


};

}//namespace

#endif //PCALIBRA_CORE_H
