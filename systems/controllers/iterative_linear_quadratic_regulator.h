#pragma once

#include <memory>
#include <vector>

#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {
namespace controllers {

struct StageCostExpansion {
  Eigen::MatrixXd gradJxx;
  Eigen::VectorXd gradJx;
  Eigen::MatrixXd gradJuu;
  Eigen::VectorXd gradJu;
};

/// A structure for storing the result of iLQR
struct IterativeLinearQuadraticRegulatorResult {
  Eigen::MatrixXd X;
  Eigen::MatrixXd U;
  double J;
};

struct BackwardPassResult {
    Eigen::MatrixXd K;
    Eigen::MatrixXd d;
    double DeltaJ;
};

struct ForwardPassResult {
  Eigen::MatrixXd X;
  Eigen::MatrixXd U;
  double J;
};

// Add some helper functions for iLQR. Might include BackwardPass and ForwardPass
// and other functions for intermediate calculations...

/**
* @brief Compute and return the stage cost, sometimes referred to as running cost,
*        for a quadratic cost function.
*/
double GetStageCostAtDiscreteTimeStep(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& u,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const Eigen::VectorXd& xref,
    const Eigen::VectorXd& uref);

/**
* @brief Compute and return the hessians and gradients of a quadratic cost function
*        and store them in a StageCostExpansion structure.
*/
StageCostExpansion GetStageCostExpansionAtDiscreteTimeStep(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& u,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const Eigen::VectorXd& xref,
    const Eigen::VectorXd& uref
);

/// Computes an optimal state and control trajectory using discrete iterative LQR
IterativeLinearQuadraticRegulatorResult IterativeLinearQuadraticRegulator(
  const Eigen::Ref<const Eigen::MatrixXd>& A,
  const Eigen::Ref<const Eigen::MatrixXd>& B,
  const Eigen::Ref<const Eigen::MatrixXd>& Q,
  const Eigen::Ref<const Eigen::MatrixXd>& R,
  const Eigen::Ref<const Eigen::MatrixXd>& N,
  const std::vector<const Eigen::Ref<const Eigen::VectorXd>>& Xref,
  const std::vector<const Eigen::Ref<const Eigen::VectorXd>>& Uref,
  const std::vector<const Eigen::Ref<const Eigen::VectorXd>>& X,
  const std::vector<const Eigen::Ref<const Eigen::VectorXd>>& U
);

// std::unique_ptr<LinearSystem<double>> IterativeLinearQuadraticRegulator(
//     const LinearSystem<double>& system,
//     const Eigen::Ref<const Eigen::MatrixXd>& Q,
//     const Eigen::Ref<const Eigen::MatrixXd>& R,
//     const Eigen::Ref<const Eigen::MatrixXd>& N =
//     Eigen::Matrix<double, 0, 0>::Zero());

}  // namespace controllers
}  // namespace systems
}  // namespace drake
