#include "drake/systems/controllers/iterative_linear_quadratic_regulator.h"
#include "drake/systems/analysis/simulator.h"

#include <vector>
#include <Eigen/QR>
#include <Eigen/StdVector>

namespace drake {
namespace systems {
namespace controllers {

double GetStageCostAtDiscreteTimeStep(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& u,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const Eigen::VectorXd& xref,
    const Eigen::VectorXd& uref)
{
  Eigen::VectorXd dx = x - xref;
  Eigen::VectorXd du = u - uref;
  Eigen::VectorXd rhs_x = Q*dx;
  Eigen::VectorXd rhs_u = R*du;

  double stage_cost_at_k = 0.5*rhs_x.dot(dx) + 0.5*rhs_u.dot(du);
  return stage_cost_at_k;
}

StageCostExpansion GetStageCostExpansionAtDiscreteTimeStep(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& u,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const Eigen::VectorXd& xref,
    const Eigen::VectorXd& uref)
{
  StageCostExpansion expansion;
  expansion.gradJxx = Q;
  expansion.gradJx = Q*(x - xref);
  expansion.gradJuu = R;
  expansion.gradJu = R*(u - uref);
  return expansion;
}

double GetTrajectoryCost(
    const int& num_inputs,
    const int& num_states,
    const std::vector<const Eigen::Ref<const Eigen::VectorXd>>& X,
    const std::vector<const Eigen::Ref<const Eigen::VectorXd>>& U,
    const std::vector<const Eigen::Ref<const Eigen::VectorXd>>& Xref,
    const std::vector<const Eigen::Ref<const Eigen::VectorXd>>& Uref,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const int& N)
{
  double cost = 0.0;
  for (int k = 0; k < N-1; k++)
  {
    cost += GetStageCostAtDiscreteTimeStep(X[k], U[k], Q, R, Xref[k], Uref[k]);
  }

  // TODO(blakeb): Incorporate Qf and Rf
  // Add terminal cost
  cost += GetStageCostAtDiscreteTimeStep(X[N], Eigen::VectorXd::Zero(num_inputs), Q, R, Xref[N], Eigen::VectorXd::Zero(num_inputs));

  return cost;
}

BackwardPassResult BackwardPass(
    const int& num_inputs,
    const int& num_states,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const std::vector<const Eigen::Ref<const Eigen::VectorXd>>& X,
    const std::vector<const Eigen::Ref<const Eigen::VectorXd>>& U,
    const std::vector<const Eigen::Ref<const Eigen::VectorXd>>& Xref,
    const std::vector<const Eigen::Ref<const Eigen::VectorXd>>& Uref,
    double& DeltaJ,
    const int& N)
{
  // Nothing here yet
  BackwardPassResult bp_result;

  std::vector<Eigen::MatrixXd> P(N);
  std::vector<Eigen::VectorXd> p(N);
  std::vector<Eigen::VectorXd> d(N-1);
  std::vector<Eigen::MatrixXd> K(N-1);

  bp_result.DeltaJ = 0.0;

  StageCostExpansion result = GetStageCostExpansionAtDiscreteTimeStep(X[N], Eigen::VectorXd::Zero(num_inputs),  Q, R, Xref[N], Eigen::VectorXd::Zero(num_inputs));
  P[N] = result.gradJxx;
  p[N] = result.gradJx;

  for (int k = N-1; k >= 0; k--)
  {
    // Need too compute the Jacobians of the dynamics somehow...
    result = GetStageCostExpansionAtDiscreteTimeStep(X[k], U[k], Q, R, Xref[k], Uref[k]);
    Eigen::VectorXd Qx = result.gradJx; // + A.transpose()*p[k+1]
    Eigen::VectorXd Qu = result.gradJu; // + B.transpose()*p[k+1]

    Eigen::MatrixXd Qxx = result.gradJxx; // + A.transpose()*P[k+1]*A
    Eigen::MatrixXd Quu = result.gradJuu; // + B.transpose()*P[k+1]*B
    // Need to add Qux and Qxu, let's start with zeros
    Eigen::MatrixXd Qux = Eigen::MatrixXd::Zero(num_states, num_states);
    Eigen::MatrixXd Qxu = Eigen::MatrixXd::Zero(num_states, num_states);

    K[k] = Quu.inverse()*Qux;
    d[k] = Quu.inverse()*Qu;

    P[k] = Qxx + K[k].transpose()*Quu*K[k] - K[k].transpose()*Qux - Qxu*K[k];
    p[k] = Qx + K[k].transpose()*Quu*d[k] - K[k].transpose()*Qu - Qxu*d[k];

    bp_result.DeltaJ = Qu.dot(d[k]);
  }

  Eigen::MatrixXd P_init(num_states, num_states);

  return bp_result;
}

ForwardPassResult ForwardPass(
  const std::vector<const Eigen::Ref<const Eigen::VectorXd>>& X,
  const std::vector<const Eigen::Ref<const Eigen::VectorXd>>& U,
  const std::vector<const Eigen::Ref<const Eigen::MatrixXd>>& K, 
  const std::vector<const Eigen::Ref<const Eigen::VectorXd>>& d,
  double& DeltaJ)
{
  // Nothing here yet
  ForwardPassResult fp_result;
  fp_result.X = X;
  return fp_result;
}

IterativeLinearQuadraticRegulatorResult IterativeLinearQuadraticRegulator(
  const Eigen::Ref<const Eigen::MatrixXd>& A,
  const Eigen::Ref<const Eigen::MatrixXd>& B,
  const Eigen::Ref<const Eigen::MatrixXd>& Q,
  const Eigen::Ref<const Eigen::MatrixXd>& R,
  const Eigen::Ref<const Eigen::MatrixXd>& N,
  const std::vector<const Eigen::Ref<const Eigen::VectorXd>>& Xref,
  const std::vector<const Eigen::Ref<const Eigen::VectorXd>>& Uref,
  const std::vector<const Eigen::Ref<const Eigen::VectorXd>>& X,
  const std::vector<const Eigen::Ref<const Eigen::VectorXd>>& U)
 {

  BackwardPassResult bp_result;
  ForwardPassResult fp_result;

  // Initialize X0, U, and tolerance
  // Simulate from X0 using U (need a forward rollout)

  // compute J (quadratic cost function) using X and U
  bp_result.DeltaJ = 0.0;

  IterativeLinearQuadraticRegulatorResult result;
  result.X = fp_result.X;
  result.U = fp_result.U;
  result.J = fp_result.J;

  return result;
}

std::unique_ptr<systems::AffineSystem<double>> IterativeLinearQuadraticRegulator(
    const System<double>& system,
    const Context<double>& context,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const Eigen::Ref<const Eigen::MatrixXd>& N,
    int input_port_index) {
  
  // Instantiate a simulator using system and context
  Simulator<double> simulator(system);

  // Pass this simulator into the IterativeLinearQuadraticRegulator function so that
  // ForwardPass can use it for simulating forward the trajectory...

  return nullptr;
}


}  // namespace controllers
}  // namespace systems
}  // namespace drake
