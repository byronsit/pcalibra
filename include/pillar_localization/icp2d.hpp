//
// Created by bohuan on 19-6-21.
//

#ifndef PILLAR_LOCALIZATION_ICP2D_HPP
#define PILLAR_LOCALIZATION_ICP2D_HPP


// PCL includes
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/registration/registration.h>
//#include <pcl/registration/transformation_estimation_svd.h>
//#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/default_convergence_criteria.h>

#include <pcl/registration/boost.h>
#include <pcl/correspondence.h>
#include <pcl/registration/transformation_estimation_2D.h>

namespace pcl {
/** \brief @b IterativeClosestPoint provides a base implementation of the Iterative Closest Point algorithm.
  * The transformation is estimated based on Singular Value Decomposition (SVD).
  *
  * The algorithm has several termination criteria:
  *
  * <ol>
  * <li>Number of iterations has reached the maximum user imposed number of iterations (via \ref setMaximumIterations)</li>
  * <li>The epsilon (difference) between the previous transformation and the current estimated transformation is smaller than an user imposed value (via \ref setTransformationEpsilon)</li>
  * <li>The sum of Euclidean squared errors is smaller than a user defined threshold (via \ref setEuclideanFitnessEpsilon)</li>
  * </ol>
  *
  *
  * Usage example:
  * \code
  * IterativeClosestPoint<PointXYZ, PointXYZ> icp;
  * // Set the input source and target
  * icp.setInputCloud (cloud_source);
  * icp.setInputTarget (cloud_target);
  *
  * // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  * icp.setMaxCorrespondenceDistance (0.05);
  * // Set the maximum number of iterations (criterion 1)
  * icp.setMaximumIterations (50);
  * // Set the transformation epsilon (criterion 2)
  * icp.setTransformationEpsilon (1e-8);
  * // Set the euclidean distance difference epsilon (criterion 3)
  * icp.setEuclideanFitnessEpsilon (1);
  *
  * // Perform the alignment
  * icp.align (cloud_source_registered);
  *
  * // Obtain the transformation that aligned cloud_source to cloud_source_registered
  * Eigen::Matrix4f transformation = icp.getFinalTransformation ();
  * \endcode
  *
  * \author Radu B. Rusu, Michael Dixon
  * \ingroup registration
  */
template<typename PointSource, typename PointTarget, typename Scalar = float>
class IterativeClosestPoint2D : public Registration<PointSource, PointTarget, Scalar> {
 public:
  typedef typename Registration<PointSource, PointTarget, Scalar>::PointCloudSource PointCloudSource;
  typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
  typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

  typedef typename Registration<PointSource, PointTarget, Scalar>::PointCloudTarget PointCloudTarget;
  typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
  typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

  typedef PointIndices::Ptr PointIndicesPtr;
  typedef PointIndices::ConstPtr PointIndicesConstPtr;

  typedef boost::shared_ptr<IterativeClosestPoint2D<PointSource, PointTarget, Scalar> > Ptr;
  typedef boost::shared_ptr<const IterativeClosestPoint2D<PointSource, PointTarget, Scalar> > ConstPtr;

  using Registration<PointSource, PointTarget, Scalar>::reg_name_;
  using Registration<PointSource, PointTarget, Scalar>::getClassName;
  using Registration<PointSource, PointTarget, Scalar>::input_;
  using Registration<PointSource, PointTarget, Scalar>::indices_;
  using Registration<PointSource, PointTarget, Scalar>::target_;
  using Registration<PointSource, PointTarget, Scalar>::nr_iterations_;
  using Registration<PointSource, PointTarget, Scalar>::max_iterations_;
  using Registration<PointSource, PointTarget, Scalar>::previous_transformation_;
  using Registration<PointSource, PointTarget, Scalar>::final_transformation_;
  using Registration<PointSource, PointTarget, Scalar>::transformation_;
  using Registration<PointSource, PointTarget, Scalar>::transformation_epsilon_;
  using Registration<PointSource, PointTarget, Scalar>::converged_;
  using Registration<PointSource, PointTarget, Scalar>::corr_dist_threshold_;
  using Registration<PointSource, PointTarget, Scalar>::inlier_threshold_;
  using Registration<PointSource, PointTarget, Scalar>::min_number_correspondences_;
  using Registration<PointSource, PointTarget, Scalar>::update_visualizer_;
  using Registration<PointSource, PointTarget, Scalar>::euclidean_fitness_epsilon_;
  using Registration<PointSource, PointTarget, Scalar>::correspondences_;
  using Registration<PointSource, PointTarget, Scalar>::transformation_estimation_;
  using Registration<PointSource, PointTarget, Scalar>::correspondence_estimation_;
  using Registration<PointSource, PointTarget, Scalar>::correspondence_rejectors_;

  typename pcl::registration::DefaultConvergenceCriteria<Scalar>::Ptr convergence_criteria_;
  typedef typename Registration<PointSource, PointTarget, Scalar>::Matrix4 Matrix4;

  /** \brief Empty constructor. */
  IterativeClosestPoint2D()
      : x_idx_offset_(0),
        y_idx_offset_(0),
        z_idx_offset_(0),
        nx_idx_offset_(0),
        ny_idx_offset_(0),
        nz_idx_offset_(0),
        use_reciprocal_correspondence_(false),
        source_has_normals_(false),
        target_has_normals_(false) {
    reg_name_ = "IterativeClosestPoint2D";
    //transformation_estimation_.reset(new pcl::registration::TransformationEstimationSVD<PointSource,
    //                                                                                    PointTarget,
    //                                                                                    Scalar>());

    transformation_estimation_.reset(new pcl::registration::TransformationEstimation2D<PointSource,
                                                                                        PointTarget,
                                                                                        Scalar>());
    correspondence_estimation_.reset(new pcl::registration::CorrespondenceEstimation<PointSource, PointTarget, Scalar>);
    convergence_criteria_.reset(new pcl::registration::DefaultConvergenceCriteria<Scalar>(nr_iterations_,
                                                                                          transformation_,
                                                                                          *correspondences_));
  };

  /** \brief Empty destructor */
  virtual ~IterativeClosestPoint2D() {}

  /** \brief Returns a pointer to the DefaultConvergenceCriteria used by the IterativeClosestPoint class.
    * This allows to check the convergence state after the align() method as well as to configure
    * DefaultConvergenceCriteria's parameters not available through the ICP API before the align()
    * method is called. Please note that the align method sets max_iterations_,
    * euclidean_fitness_epsilon_ and transformation_epsilon_ and therefore overrides the default / set
    * values of the DefaultConvergenceCriteria instance.
    * \return Pointer to the IterativeClosestPoint's DefaultConvergenceCriteria.
    */
  inline typename pcl::registration::DefaultConvergenceCriteria<Scalar>::Ptr
  getConvergeCriteria() {
    return convergence_criteria_;
  }

  /** \brief Provide a pointer to the input source
    * (e.g., the point cloud that we want to align to the target)
    *
    * \param[in] cloud the input point cloud source
    */
  virtual void
  setInputSource(const PointCloudSourceConstPtr &cloud) {
    Registration<PointSource, PointTarget, Scalar>::setInputSource(cloud);
    std::vector<pcl::PCLPointField> fields;
    pcl::getFields(*cloud, fields);
    source_has_normals_ = false;
    for (size_t i = 0; i < fields.size(); ++i) {
      if (fields[i].name == "x") x_idx_offset_ = fields[i].offset;
      else if (fields[i].name == "y") y_idx_offset_ = fields[i].offset;
      else if (fields[i].name == "z") z_idx_offset_ = fields[i].offset;
      else if (fields[i].name == "normal_x") {
        source_has_normals_ = true;
        nx_idx_offset_ = fields[i].offset;
      } else if (fields[i].name == "normal_y") {
        source_has_normals_ = true;
        ny_idx_offset_ = fields[i].offset;
      } else if (fields[i].name == "normal_z") {
        source_has_normals_ = true;
        nz_idx_offset_ = fields[i].offset;
      }
    }
  }

  /** \brief Provide a pointer to the input target
    * (e.g., the point cloud that we want to align to the target)
    *
    * \param[in] cloud the input point cloud target
    */
  virtual void
  setInputTarget(const PointCloudTargetConstPtr &cloud) {
    Registration<PointSource, PointTarget, Scalar>::setInputTarget(cloud);
    std::vector<pcl::PCLPointField> fields;
    pcl::getFields(*cloud, fields);
    target_has_normals_ = false;
    for (size_t i = 0; i < fields.size(); ++i) {
      if (fields[i].name == "normal_x" || fields[i].name == "normal_y" || fields[i].name == "normal_z") {
        target_has_normals_ = true;
        break;
      }
    }
  }

  /** \brief Set whether to use reciprocal correspondence or not
    *
    * \param[in] use_reciprocal_correspondence whether to use reciprocal correspondence or not
    */
  inline void
  setUseReciprocalCorrespondences(bool use_reciprocal_correspondence) {
    use_reciprocal_correspondence_ = use_reciprocal_correspondence;
  }

  /** \brief Obtain whether reciprocal correspondence are used or not */
  inline bool
  getUseReciprocalCorrespondences() const {
    return (use_reciprocal_correspondence_);
  }

 protected:

  /** \brief Apply a rigid transform to a given dataset. Here we check whether whether
    * the dataset has surface normals in addition to XYZ, and rotate normals as well.
    * \param[in] input the input point cloud
    * \param[out] output the resultant output point cloud
    * \param[in] transform a 4x4 rigid transformation
    * \note Can be used with cloud_in equal to cloud_out
    */
  virtual void
  transformCloud(const PointCloudSource &input,
                 PointCloudSource &output,
                 const Matrix4 &transform);

  /** \brief Rigid transformation computation method  with initial guess.
    * \param output the transformed input point cloud dataset using the rigid transformation found
    * \param guess the initial guess of the transformation to compute
    */
  virtual void
  computeTransformation(PointCloudSource &output, const Matrix4 &guess);

  /** \brief Looks at the Estimators and Rejectors and determines whether their blob-setter methods need to be called */
  virtual void
  determineRequiredBlobData();

  /** \brief XYZ fields offset. */
  size_t x_idx_offset_, y_idx_offset_, z_idx_offset_;

  /** \brief Normal fields offset. */
  size_t nx_idx_offset_, ny_idx_offset_, nz_idx_offset_;

  /** \brief The correspondence type used for correspondence estimation. */
  bool use_reciprocal_correspondence_;

  /** \brief Internal check whether source dataset has normals or not. */
  bool source_has_normals_;
  /** \brief Internal check whether target dataset has normals or not. */
  bool target_has_normals_;

  /** \brief Checks for whether estimators and rejectors need various data */
  bool need_source_blob_, need_target_blob_;
};
} // namespace pcl

#include <pcl/registration/boost.h>
#include <pcl/correspondence.h>

///////////////////////////////////////////////////////////////////////////////////////////
template<typename PointSource, typename PointTarget, typename Scalar>
void pcl::IterativeClosestPoint2D<PointSource, PointTarget, Scalar>::transformCloud(
    const PointCloudSource &input,
    PointCloudSource &output,
    const Matrix4 &transform) {
  Eigen::Vector4f pt(0.0f, 0.0f, 0.0f, 1.0f), pt_t;
  Eigen::Matrix4f tr = transform.template cast<float>();

  // XYZ is ALWAYS present due to the templatization, so we only have to check for normals
  if (source_has_normals_) {
    Eigen::Vector3f nt, nt_t;
    Eigen::Matrix3f rot = tr.block<3, 3>(0, 0);

    for (size_t i = 0; i < input.size(); ++i) {
      const uint8_t *data_in = reinterpret_cast<const uint8_t *> (&input[i]);
      uint8_t *data_out = reinterpret_cast<uint8_t *> (&output[i]);
      memcpy(&pt[0], data_in + x_idx_offset_, sizeof(float));
      memcpy(&pt[1], data_in + y_idx_offset_, sizeof(float));
      memcpy(&pt[2], data_in + z_idx_offset_, sizeof(float));

      if (!std::isfinite(pt[0]) || !std::isfinite(pt[1]) || !std::isfinite(pt[2]))
        continue;

      pt_t = tr * pt;

      memcpy(data_out + x_idx_offset_, &pt_t[0], sizeof(float));
      memcpy(data_out + y_idx_offset_, &pt_t[1], sizeof(float));
      memcpy(data_out + z_idx_offset_, &pt_t[2], sizeof(float));

      memcpy(&nt[0], data_in + nx_idx_offset_, sizeof(float));
      memcpy(&nt[1], data_in + ny_idx_offset_, sizeof(float));
      memcpy(&nt[2], data_in + nz_idx_offset_, sizeof(float));

      if (!std::isfinite(nt[0]) || !std::isfinite(nt[1]) || !std::isfinite(nt[2]))
        continue;

      nt_t = rot * nt;

      memcpy(data_out + nx_idx_offset_, &nt_t[0], sizeof(float));
      memcpy(data_out + ny_idx_offset_, &nt_t[1], sizeof(float));
      memcpy(data_out + nz_idx_offset_, &nt_t[2], sizeof(float));
    }
  } else {
    for (size_t i = 0; i < input.size(); ++i) {
      const uint8_t *data_in = reinterpret_cast<const uint8_t *> (&input[i]);
      uint8_t *data_out = reinterpret_cast<uint8_t *> (&output[i]);
      memcpy(&pt[0], data_in + x_idx_offset_, sizeof(float));
      memcpy(&pt[1], data_in + y_idx_offset_, sizeof(float));
      memcpy(&pt[2], data_in + z_idx_offset_, sizeof(float));

      if (!std::isfinite(pt[0]) || !std::isfinite(pt[1]) || !std::isfinite(pt[2]))
        continue;

      pt_t = tr * pt;

      memcpy(data_out + x_idx_offset_, &pt_t[0], sizeof(float));
      memcpy(data_out + y_idx_offset_, &pt_t[1], sizeof(float));
      memcpy(data_out + z_idx_offset_, &pt_t[2], sizeof(float));
    }
  }

}

///////////////////////////////////////////////////////////////////////////////////////////
template<typename PointSource, typename PointTarget, typename Scalar>
void
pcl::IterativeClosestPoint2D<PointSource, PointTarget, Scalar>::computeTransformation(
    PointCloudSource &output, const Matrix4 &guess) {
  // Point cloud containing the correspondences of each point in <input, indices>
  PointCloudSourcePtr input_transformed(new PointCloudSource);

  nr_iterations_ = 0;
  converged_ = false;

  // Initialise final transformation to the guessed one
  final_transformation_ = guess;

  // If the guessed transformation is non identity
  if (guess != Matrix4::Identity()) {
    input_transformed->resize(input_->size());
    // Apply guessed transformation prior to search for neighbours
    transformCloud(*input_, *input_transformed, guess);
  } else
    *input_transformed = *input_;

  transformation_ = Matrix4::Identity();

  // Make blobs if necessary
  determineRequiredBlobData();
  PCLPointCloud2::Ptr target_blob(new PCLPointCloud2);
  if (need_target_blob_)
    pcl::toPCLPointCloud2(*target_, *target_blob);

  // Pass in the default target for the Correspondence Estimation/Rejection code
  correspondence_estimation_->setInputTarget(target_);
  if (correspondence_estimation_->requiresTargetNormals())
    correspondence_estimation_->setTargetNormals(target_blob);
  // Correspondence Rejectors need a binary blob
  for (size_t i = 0; i < correspondence_rejectors_.size(); ++i) {
    registration::CorrespondenceRejector::Ptr &rej = correspondence_rejectors_[i];
    if (rej->requiresTargetPoints())
      rej->setTargetPoints(target_blob);
    if (rej->requiresTargetNormals() && target_has_normals_)
      rej->setTargetNormals(target_blob);
  }

  convergence_criteria_->setMaximumIterations(max_iterations_);
  convergence_criteria_->setRelativeMSE(euclidean_fitness_epsilon_);
  convergence_criteria_->setTranslationThreshold(transformation_epsilon_);
  convergence_criteria_->setRotationThreshold(1.0 - transformation_epsilon_);

  // Repeat until convergence
  do {
    // Get blob data if needed
    PCLPointCloud2::Ptr input_transformed_blob;
    if (need_source_blob_) {
      input_transformed_blob.reset(new PCLPointCloud2);
      toPCLPointCloud2(*input_transformed, *input_transformed_blob);
    }
    // Save the previously estimated transformation
    previous_transformation_ = transformation_;

    // Set the source each iteration, to ensure the dirty flag is updated
    correspondence_estimation_->setInputSource(input_transformed);
    if (correspondence_estimation_->requiresSourceNormals())
      correspondence_estimation_->setSourceNormals(input_transformed_blob);
    // Estimate correspondences
    if (use_reciprocal_correspondence_)
      correspondence_estimation_->determineReciprocalCorrespondences(*correspondences_, corr_dist_threshold_);
    else
      correspondence_estimation_->determineCorrespondences(*correspondences_, corr_dist_threshold_);

    //if (correspondence_rejectors_.empty ())
    CorrespondencesPtr temp_correspondences(new Correspondences(*correspondences_));
    for (size_t i = 0; i < correspondence_rejectors_.size(); ++i) {
      registration::CorrespondenceRejector::Ptr &rej = correspondence_rejectors_[i];
      PCL_DEBUG ("Applying a correspondence rejector method: %s.\n", rej->getClassName().c_str());
      if (rej->requiresSourcePoints())
        rej->setSourcePoints(input_transformed_blob);
      if (rej->requiresSourceNormals() && source_has_normals_)
        rej->setSourceNormals(input_transformed_blob);
      rej->setInputCorrespondences(temp_correspondences);
      rej->getCorrespondences(*correspondences_);
      // Modify input for the next iteration
      if (i < correspondence_rejectors_.size() - 1)
        *temp_correspondences = *correspondences_;
    }

    size_t cnt = correspondences_->size();
    // Check whether we have enough correspondences
    if (static_cast<int> (cnt) < min_number_correspondences_) {
      PCL_ERROR ("[pcl::%s::computeTransformation] Not enough correspondences found. Relax your threshold parameters.\n",
                 getClassName().c_str());
      convergence_criteria_->setConvergenceState(pcl::registration::DefaultConvergenceCriteria<Scalar>::CONVERGENCE_CRITERIA_NO_CORRESPONDENCES);
      converged_ = false;
      break;
    }

    // Estimate the transform
    transformation_estimation_->estimateRigidTransformation(*input_transformed,
                                                            *target_,
                                                            *correspondences_,
                                                            transformation_);

    // Transform the data
    transformCloud(*input_transformed, *input_transformed, transformation_);

    // Obtain the final transformation
    final_transformation_ = transformation_ * final_transformation_;

    ++nr_iterations_;

    // Update the vizualization of icp convergence
    //if (update_visualizer_ != 0)
    //  update_visualizer_(output, source_indices_good, *target_, target_indices_good );

    converged_ = static_cast<bool> ((*convergence_criteria_));
  } while (convergence_criteria_->getConvergenceState()
      == pcl::registration::DefaultConvergenceCriteria<Scalar>::CONVERGENCE_CRITERIA_NOT_CONVERGED);

  // Transform the input cloud using the final transformation
  PCL_DEBUG (
      "Transformation is:\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n",
      final_transformation_(0, 0),
      final_transformation_(0, 1),
      final_transformation_(0, 2),
      final_transformation_(0, 3),
      final_transformation_(1, 0),
      final_transformation_(1, 1),
      final_transformation_(1, 2),
      final_transformation_(1, 3),
      final_transformation_(2, 0),
      final_transformation_(2, 1),
      final_transformation_(2, 2),
      final_transformation_(2, 3),
      final_transformation_(3, 0),
      final_transformation_(3, 1),
      final_transformation_(3, 2),
      final_transformation_(3, 3));

  // Copy all the values
  output = *input_;
  // Transform the XYZ + normals
  transformCloud(*input_, output, final_transformation_);
}

template<typename PointSource, typename PointTarget, typename Scalar>
void
pcl::IterativeClosestPoint2D<PointSource, PointTarget, Scalar>::determineRequiredBlobData() {
  need_source_blob_ = false;
  need_target_blob_ = false;
  // Check estimator
  need_source_blob_ |= correspondence_estimation_->requiresSourceNormals();
  need_target_blob_ |= correspondence_estimation_->requiresTargetNormals();
  // Add warnings if necessary
  if (correspondence_estimation_->requiresSourceNormals() && !source_has_normals_) {
    PCL_WARN("[pcl::%s::determineRequiredBlobData] Estimator expects source normals, but we can't provide them.\n",
             getClassName().c_str());
  }
  if (correspondence_estimation_->requiresTargetNormals() && !target_has_normals_) {
    PCL_WARN("[pcl::%s::determineRequiredBlobData] Estimator expects target normals, but we can't provide them.\n",
             getClassName().c_str());
  }
  // Check rejectors
  for (size_t i = 0; i < correspondence_rejectors_.size(); i++) {
    registration::CorrespondenceRejector::Ptr &rej = correspondence_rejectors_[i];
    need_source_blob_ |= rej->requiresSourcePoints();
    need_source_blob_ |= rej->requiresSourceNormals();
    need_target_blob_ |= rej->requiresTargetPoints();
    need_target_blob_ |= rej->requiresTargetNormals();
    if (rej->requiresSourceNormals() && !source_has_normals_) {
      PCL_WARN("[pcl::%s::determineRequiredBlobData] Rejector %s expects source normals, but we can't provide them.\n",
               getClassName().c_str(),
               rej->getClassName().c_str());
    }
    if (rej->requiresTargetNormals() && !target_has_normals_) {
      PCL_WARN("[pcl::%s::determineRequiredBlobData] Rejector %s expects target normals, but we can't provide them.\n",
               getClassName().c_str(),
               rej->getClassName().c_str());
    }
  }
}



#endif //PILLAR_LOCALIZATION_ICP2D_HPP
