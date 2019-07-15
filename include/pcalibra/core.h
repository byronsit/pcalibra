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
   * @brief 计算correspondence
   * 主要利用pcl的方式，这里简单的填充一些东西
   * 只考虑point和line的对应情况*/
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
    CHECK_NE(reference_, NULL) << "have not set reference\n";
    CHECK_NE(aligned_, NULL) << "have not set aligned\n";

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




 private:
  std::vector<Correspondence> cor_;
  pcl::pclpc3i* reference_;
  pcl::pclpc3i* aligned_;
  std::vector<ELine> lines_;
  std::vector<pcl::pclpc3> pcs_;
  Eigen::Isometry3d isometry;


};

}//namespace

#endif //PCALIBRA_CORE_H
