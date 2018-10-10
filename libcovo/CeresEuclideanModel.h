#include "ceres/ceres.h"
#include <Eigen/Dense>

struct EuclideanErrorResidual{

    EuclideanErrorResidual(
        const double& frame_1_x, const double& frame_1_y, const double& frame_1_z,
        const double& frame_2_x, const double& frame_2_y, const double& frame_2_z,
        const Eigen::Matrix<double, 3, 3> &sqrt_information):
      _frame_1_x(frame_1_x), _frame_1_y(frame_1_y), _frame_1_z(frame_1_z),
      _frame_2_x(frame_2_x), _frame_2_y(frame_2_y), _frame_2_z(frame_2_z),
      _sqrt_information(sqrt_information) {}

    template <typename T>
    bool operator()(
        const T *const p_a_ptr, 
        const T *const q_a_ptr, 
        T* residuals_ptr) const {

        Eigen::Map<const Eigen::Matrix<T, 3, 1> > p_a(p_a_ptr);
        Eigen::Map<const Eigen::Quaternion<T> > q_a(q_a_ptr);

        Eigen::Matrix<T, 3, 1> tmp_point_cloud;
        tmp_point_cloud(0) = T(_frame_2_x);
        tmp_point_cloud(1) = T(_frame_2_y);
        tmp_point_cloud(2) = T(_frame_2_z);

        // rotate
        Eigen::Matrix<T, 3, 1> transformed_point = q_a * tmp_point_cloud;
        // translate
        transformed_point += p_a;

        // calc residuals
        Eigen::Map<Eigen::Matrix<T, 3, 1> > residuals(residuals_ptr);
        residuals(0) = transformed_point[0] - _frame_1_x;
        residuals(1) = transformed_point[1] - _frame_1_y;
        residuals(2) = transformed_point[2] - _frame_1_z;

        // apply info matrix (weights)
        residuals.applyOnTheLeft(_sqrt_information.template cast<T>());
        return true;
    }

    static ceres::CostFunction* Create(
        const double& frame_1_x, const double& frame_1_y, const double& frame_1_z,
        const double& frame_2_x, const double& frame_2_y, const double& frame_2_z,
        const Eigen::Matrix<double, 3, 3> &sqrt_information) 
    {
      return (new ceres::AutoDiffCostFunction<EuclideanErrorResidual, 3, 3, 4>(
            new EuclideanErrorResidual(
              frame_1_x, frame_1_y, frame_1_z,
              frame_2_x, frame_2_y, frame_2_z,
              sqrt_information)));
    }

    const double _frame_1_x, _frame_1_y, _frame_1_z;
    const double _frame_2_x, _frame_2_y, _frame_2_z;

    const Eigen::Matrix<double, 3, 3> _sqrt_information;

};



