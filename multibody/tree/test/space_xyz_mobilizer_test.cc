#include "drake/multibody/tree/space_xyz_mobilizer.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/tree/ball_rpy_joint.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/test/mobilizer_tester.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using Eigen::Matrix3d;
using Eigen::Vector3d;
using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrixd;
using std::make_unique;
using std::unique_ptr;
using systems::Context;

constexpr double kTolerance = 10 * std::numeric_limits<double>::epsilon();

// Fixture to setup a simple MBT model containing a space xyz mobilizer.
class SpaceXYZMobilizerTest :  public MobilizerTester {
 public:
  // Creates a simple model consisting of a single body with a space xyz
  // mobilizer connecting it to the world.
  void SetUp() override {
    mobilizer_ = &AddJointAndFinalize<BallRpyJoint, SpaceXYZMobilizer>(
        std::make_unique<BallRpyJoint<double>>("joint0",
            tree().world_body().body_frame(), body_->body_frame()));
  }

 protected:
  const SpaceXYZMobilizer<double>* mobilizer_{nullptr};
};

TEST_F(SpaceXYZMobilizerTest, CanRotateOrTranslate) {
  EXPECT_TRUE(mobilizer_->can_rotate());
  EXPECT_FALSE(mobilizer_->can_translate());
}

// Verifies methods to mutate and access the context.
TEST_F(SpaceXYZMobilizerTest, StateAccess) {
  const Vector3d rpy_value(M_PI / 3, -M_PI / 3, M_PI / 5);
  mobilizer_->set_angles(context_.get(), rpy_value);
  EXPECT_EQ(mobilizer_->get_angles(*context_), rpy_value);

  // Set mobilizer orientation using a rotation matrix.
  const RollPitchYawd rpy(M_PI / 5, -M_PI / 7, M_PI / 3);
  const RotationMatrixd R_WB(rpy);
  mobilizer_->SetFromRotationMatrix(context_.get(), R_WB);
  EXPECT_TRUE(CompareMatrices(
    mobilizer_->get_angles(*context_), rpy.vector(),
    kTolerance, MatrixCompareType::relative));
}

TEST_F(SpaceXYZMobilizerTest, ZeroState) {
  // Set an arbitrary "non-zero" state.
  const Vector3d rpy_value(M_PI / 3, -M_PI / 3, M_PI / 5);
  mobilizer_->set_angles(context_.get(), rpy_value);
  EXPECT_EQ(mobilizer_->get_angles(*context_), rpy_value);

  // Set the "zero state" for this mobilizer, which does happen to be that of
  // an identity rigid transform.
  mobilizer_->set_zero_state(*context_, &context_->get_mutable_state());
  const RigidTransformd X_WB(
      mobilizer_->CalcAcrossMobilizerTransform(*context_));
  EXPECT_TRUE(X_WB.IsExactlyIdentity());
}

// For an arbitrary state verify that the computed Nplus(q) matrix is the
// inverse of N(q).
TEST_F(SpaceXYZMobilizerTest, KinematicMapping) {
  const Vector3d rpy(M_PI / 3, -M_PI / 3, M_PI / 5);
  mobilizer_->set_angles(context_.get(), rpy);

  ASSERT_EQ(mobilizer_->num_positions(), 3);
  ASSERT_EQ(mobilizer_->num_velocities(), 3);

  // Compute N.
  MatrixX<double> N(3, 3);
  mobilizer_->CalcNMatrix(*context_, &N);

  // Compute Nplus.
  MatrixX<double> Nplus(3, 3);
  mobilizer_->CalcNplusMatrix(*context_, &Nplus);

  // Verify that Nplus is the inverse of N.
  MatrixX<double> N_x_Nplus = N * Nplus;
  MatrixX<double> Nplus_x_N = Nplus * N;

  EXPECT_TRUE(CompareMatrices(
      N_x_Nplus, Matrix3d::Identity(),
      kTolerance, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(
      Nplus_x_N, Matrix3d::Identity(),
      kTolerance, MatrixCompareType::relative));
}

TEST_F(SpaceXYZMobilizerTest, MapUsesN) {
  // Set an arbitrary "non-zero" state.
  const Vector3d rpy_value(M_PI / 3, -M_PI / 3, M_PI / 5);
  mobilizer_->set_angles(context_.get(), rpy_value);

  EXPECT_FALSE(mobilizer_->is_velocity_equal_to_qdot());

  // Set arbitrary v and MapVelocityToQDot.
  const Vector3<double> v = (Vector3<double>() << 1, 2, 3).finished();
  Vector3<double> qdot;
  mobilizer_->MapVelocityToQDot(*context_, v, &qdot);

  // Compute N.
  MatrixX<double> N(3, 3);
  mobilizer_->CalcNMatrix(*context_, &N);

  // Ensure N(q) is used in `q̇ = N(q)⋅v`
  EXPECT_TRUE(
      CompareMatrices(qdot, N * v, kTolerance, MatrixCompareType::relative));
}

TEST_F(SpaceXYZMobilizerTest, MapUsesNplus) {
  // Set an arbitrary "non-zero" state.
  const Vector3d rpy_value(M_PI / 3, -M_PI / 3, M_PI / 5);
  mobilizer_->set_angles(context_.get(), rpy_value);

  // Set arbitrary qdot and MapQDotToVelocity.
  const Vector3<double> qdot = (Vector3<double>() << 1, 2, 3).finished();
  Vector3<double> v;
  mobilizer_->MapQDotToVelocity(*context_, qdot, &v);

  // Compute Nplus.
  MatrixX<double> Nplus(3, 3);
  mobilizer_->CalcNplusMatrix(*context_, &Nplus);

  // Ensure N⁺(q) is used in `v = N⁺(q)⋅q̇`
  EXPECT_TRUE(CompareMatrices(v, Nplus * qdot, kTolerance,
                              MatrixCompareType::relative));
}

TEST_F(SpaceXYZMobilizerTest, SingularityError) {
  // Set state in singularity
  const Vector3d rpy_value(M_PI / 3, M_PI / 2, M_PI / 5);
  mobilizer_->set_angles(context_.get(), rpy_value);

  // Set arbitrary qdot and MapVelocityToQDot.
  const Vector3<double> v = (Vector3<double>() << 1, 2, 3).finished();
  Vector3<double> qdot;
  DRAKE_EXPECT_THROWS_MESSAGE(
      mobilizer_->MapVelocityToQDot(*context_, v, &qdot), ".*singularity.*");

  // Compute N.
  MatrixX<double> N(3, 3);
  DRAKE_EXPECT_THROWS_MESSAGE(mobilizer_->CalcNMatrix(*context_, &N),
                              ".*singularity.*");
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
