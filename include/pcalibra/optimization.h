/**
 * @author bohuan
 * @date 2019-07-03
 * */
#ifndef PCALIBRA_OPTIMIZATION_H
#define PCALIBRA_OPTIMIZATION_H

#include "pcalibra/utility.h"
#include "pcalibra/correspondence.h"
#include <ceres/ceres.h>
#include <Eigen/Core>

namespace pcalibra {

class Optimization {
 public:

  void PushBack(const Correspondence &cor){
    correspondence_.push_back(cor);
  }
  void Clear(){
    correspondence_.clear();
  }

  void SetCorrespondence(const std::vector<Correspondence> &cor){
    this->correspondence_ = cor;
  }

  void GetResult(Eigen::Vector3d *t, Eigen::Quaterniond *q){
    *t = this->transform_;
    *q = this->quaternion_;
  }


  /**
   * @brief 函数主要入口，调用Solve即可
   * @param 没有
   * @return true 优化成功, false 优化失败 */
  bool Solve(Eigen::Vector3d transform, Eigen::Quaterniond quaternion);

  bool Solve();

 private:
  std::vector<Correspondence> correspondence_;
  Eigen::Vector3d transform_;
  Eigen::Quaterniond quaternion_;
};

}//namespace





#endif //PCALIBRA_OPTIMIZATION_H
