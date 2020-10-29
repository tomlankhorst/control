#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "control/filter/ghk.h"

using namespace control::ghk;

TEST(ghk, param_abc) {
  auto ghk = param::abc<double>(1,2,3);
  EXPECT_DOUBLE_EQ(ghk.g, 1);
  EXPECT_DOUBLE_EQ(ghk.h, 2);
  EXPECT_DOUBLE_EQ(ghk.k, 1.5);
}

TEST(ghk, param_critical_dampened) {
  auto ghk = param::critical_dampened(0.5);
  // https://onlinelibrary.wiley.com/doi/pdf/10.1002/0471224197.ch1
  EXPECT_NEAR(ghk.g, .875, 1e-3);
  EXPECT_NEAR(ghk.h, .563, 1e-3);
  EXPECT_NEAR(ghk.k, .063, 1e-3);
}

TEST(ghk, param_optimal_gaussian_lambda) {
  auto ghk = param::optimal_gaussian(0.1);
  EXPECT_NEAR(ghk.g, .699, 1e-3);
  EXPECT_NEAR(ghk.h, .407, 1e-3);
  EXPECT_NEAR(ghk.k, .059, 1e-3);
}

TEST(ghk, param_optimal_gaussian_sig_T) {
  auto ghk = param::optimal_gaussian(1.,0.1,0.01);
  EXPECT_NEAR(ghk.g, .208, 1e-3);
  EXPECT_NEAR(ghk.h, .024, 1e-3);
  EXPECT_NEAR(ghk.k, .00071, 1e-5);
}

TEST(ghk, predict) {
  auto x0 = xva<double> { 0, 0, 0 };
  auto p = ghk<double> { 0.1, 0.01, 0.001 };
  auto [est, pred] = update_predict(p,x0,1.,1.);

  EXPECT_NEAR(est.x, .100, 1e-3);
  EXPECT_NEAR(est.v, .010, 1e-3);
  EXPECT_NEAR(est.a, .001, 1e-3);

  EXPECT_NEAR(pred.x, .111, 1e-3);
  EXPECT_NEAR(pred.v, .012, 1e-3);
  EXPECT_NEAR(pred.a, .002, 1e-3);
}