//
// Created by bohuan on 19-6-30.
//

#include "pcalibra/optimization.h"


namespace pcalibra{

/**
 * @brief ceres需要的残差类
 * */
struct CostFunctor{
  CostFunctor(const Correspondence &c):corres(c){}
  template <typename T>
  bool operator() (const T* const transform_in,
                   const T* const quaternion_in,
                   T* residual) const{
    Eigen::Map<const Eigen::Matrix<T, 3, 1> > trans(transform_in);
    Eigen::Map<const Eigen::Quaternion<T>> quat(quaternion_in);
    CHECK (corres.corres_type != None) ;
    residual[0] = this->corres.Function(trans, quat);
    return true;
  }
  Correspondence corres;
};

/**
 * @brief Optimization's core function*/
bool Optimization::Solve() {
  //new ceres::AutoDiffCostFunction<CostFunctor, 1, 3, 4>(new CostFunctor(correspondence_[0]));
  ceres::Problem problem;
  ceres::LocalParameterization* quaternion_local_parameterization =
      new ceres::EigenQuaternionParameterization;
  ceres::LossFunction *loss_function = NULL; //默认的损失函数就是求norm就行了
  std::cout<<correspondence_.size()<<std::endl;
  CHECK_GT(correspondence_.size(), 0) << "there is no correspondence";

  this->transform_.setZero();
  this->quaternion_.setIdentity();

  for (const auto &it : correspondence_) {
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<CostFunctor, 1, 3, 4>(new CostFunctor(it));
    problem.AddResidualBlock(cost_function,
                             loss_function,
                             this->transform_.data(),  //第一个优化项 transform
                             this->quaternion_.coeffs().data()); //第二个优化项 四元数

    //确保四元数合法，并且只处理3自由度问题。
    problem.SetParameterization(this->quaternion_.coeffs().data(),
                                quaternion_local_parameterization);
  }

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  options.max_num_iterations = 500;
  options.linear_solver_type = ceres::DENSE_QR;
  options.function_tolerance = 1e-10;
  options.gradient_tolerance = 1e-14;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.FullReport() << '\n';

  return summary.IsSolutionUsable();
}

}//namespace
