#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "control/filter/ghk.h"

using namespace control::ghk;

TEST(ghk, param_abc) {
  auto ghk = parameterize::abc<double>(1, 2, 3);
  EXPECT_DOUBLE_EQ(ghk.g, 1);
  EXPECT_DOUBLE_EQ(ghk.h, 2);
  EXPECT_DOUBLE_EQ(ghk.k, 1.5);
}

TEST(ghk, param_critical_dampened) {
  auto ghk = parameterize::critical_dampened(0.5);
  // https://onlinelibrary.wiley.com/doi/pdf/10.1002/0471224197.ch1
  EXPECT_NEAR(ghk.g, .875, 1e-3);
  EXPECT_NEAR(ghk.h, .563, 1e-3);
  EXPECT_NEAR(ghk.k, .063, 1e-3);
}

TEST(ghk, param_optimal_gaussian_lambda) {
  auto ghk = parameterize::optimal_gaussian(0.1);
  EXPECT_NEAR(ghk.g, .699, 1e-3);
  EXPECT_NEAR(ghk.h, .407, 1e-3);
  EXPECT_NEAR(ghk.k, .059, 1e-3);
}

TEST(ghk, param_optimal_gaussian_sig_T) {
  auto ghk = parameterize::optimal_gaussian(1., 0.1, 0.01);
  EXPECT_NEAR(ghk.g, .208, 1e-3);
  EXPECT_NEAR(ghk.h, .024, 1e-3);
  EXPECT_NEAR(ghk.k, .00071, 1e-5);
}

TEST(coeff, predict) {
  const auto x0 = state<double> {1.000000, 2.000000, 1.000000 };
  auto p = coeff<double> {1.000000, 0.010000, 0.001000/2 };
  auto err = 1e-5;
  auto check_res = [&err](result<double> ep, state<double> e, state<double> p){
    EXPECT_NEAR(ep.correction.x, e.x, err);
    EXPECT_NEAR(ep.correction.dx, e.dx, err);
    EXPECT_NEAR(ep.correction.ddx, e.ddx, err);
    EXPECT_NEAR(ep.prediction.x, p.x, err);
    EXPECT_NEAR(ep.prediction.dx, p.dx, err);
    EXPECT_NEAR(ep.prediction.ddx, p.ddx, err);
  };
  auto res = result<double> { x0, x0 };
  res = correct_predict(p, res.prediction, 1.000000, 0.100000);
  check_res(res, {1.000000,2.000000,1.000000}, {1.205000,2.100000,1.000000});
  res = correct_predict(p, res.prediction, 2.000000, 0.100000);
  check_res(res, {2.000000,2.179500,1.079500}, {2.223348,2.287450,1.079500});
  res = correct_predict(p, res.prediction, 3.000000, 0.100000);
  check_res(res, {3.000000,2.365115,1.157165}, {3.242297,2.480832,1.157165});
  res = correct_predict(p, res.prediction, 4.000000, 0.100000);
  check_res(res, {4.000000,2.556602,1.232936}, {4.261825,2.679896,1.232936});
}