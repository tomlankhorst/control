#include "control/classic/pid.h"
#include "gtest/gtest.h"
#include "gmock/gmock.h"

namespace {

typedef control::classic::P<double> CDoubleP;
class PDoubleTest : public ::testing::Test {
 protected:
  CDoubleP controller;
  PDoubleTest() : controller(2.0) {}
};

TEST_F(PDoubleTest, ProportionalZeroTest) {
  EXPECT_EQ(0.0, controller.step(0.0));
  EXPECT_EQ(0.0, controller.step(0.0));
}

TEST_F(PDoubleTest, ProportionalTest) {
  EXPECT_EQ(0.0, controller.step(0.0));
  EXPECT_EQ(2.0, controller.step(1.0));
  EXPECT_EQ(-2.0, controller.step(-1.0));
}

TEST_F(PDoubleTest, ProportionalLimitTest) {
  controller.setLimit(1.5);
  EXPECT_EQ(0.0, controller.step(0.0));
  EXPECT_EQ(1.0, controller.step(0.5));
  EXPECT_EQ(1.5, controller.step(1.0));
  EXPECT_EQ(-1.5, controller.step(-2.0));
}

// PI controller K=2, Ti=1, Ts=0.1
typedef control::classic::PI<double> CDoublePI;
class PIDoubleTest : public ::testing::Test {
 protected:
  CDoublePI controller;
  PIDoubleTest() : controller(0.1, 2.0, 1.0) {}
};


TEST_F(PIDoubleTest, LimitErrTest) {
  std::vector<double> v = {2.1, 2.3, 2.5, 2.5, 2.5};

  controller.setLimit(2.5);

  for(int i = 0; i < 5; i++)
    ASSERT_DOUBLE_EQ(controller.step(1.0), v[i]);

}

TEST_F(PIDoubleTest, ConstantErrTest) {
  std::vector<double> v = {2.1, 2.3, 2.5, 2.7, 2.9};

  for(int i = 0; i < 5; i++)
    ASSERT_DOUBLE_EQ(controller.step(1.0), v[i]);

}

TEST(PIDouble2Test, NoIntTest){
  CDoublePI controller(1.0, 2.0, control::classic::max<double>());
  std::vector<double> v= {0,2,4,6,8};
  for(int i = 0; i < 5; i++)
    ASSERT_DOUBLE_EQ(controller.step(i), v[i]);
}

// PD controller Ts=0.5, Kd=1.0, Td=1.0, N=1.0
typedef control::classic::PD<double> CDoublePD;
class PDDoubleTest : public ::testing::Test {
 protected:
  CDoublePD controller;
  PDDoubleTest() : controller(0.5, 1.0, 1.0, 1.0) {}
};


TEST_F(PDDoubleTest, SimplePDTest) {
  std::vector<double> v = {0.0, 1.8, 3.28, 4.568, 5.7408};

  for(int i = 0; i < 5; i++)
    ASSERT_DOUBLE_EQ(controller.step(i), v[i]);

  // Check that the poles are as expected (1,0.6)
  auto p = controller.poles();
  ASSERT_DOUBLE_EQ(std::get<0>(p).real(), 1);
  ASSERT_DOUBLE_EQ(std::get<0>(p).imag(), 0);
  ASSERT_DOUBLE_EQ(std::get<1>(p).real(), 0.6);
  ASSERT_DOUBLE_EQ(std::get<1>(p).imag(), 0);
}

// PID controller K=1, Ti=Inf, Td=2, N=1, Ts=1
typedef control::classic::PID<double> CDoublePID;
class PIDDoubleTest : public ::testing::Test {
 protected:
  CDoublePID controller;
  PIDDoubleTest() : controller(2.0, 1.0, 1.0, 1.0, 1.0) {}
};


TEST_F(PIDDoubleTest, SimplePID) {
  std::vector<double> u = {0, 1, 1, 2, 0};
  std::vector<double> v = {0, 2.5, 4, 8.5, 7};

  // 5 steps
  for(int i = 0; i < 5; i++)
    EXPECT_DOUBLE_EQ(controller.step(u[i]), v[i]);

}


typedef control::classic::P<float> CFloatP;
class PFloatTest : public ::testing::Test {
 protected:
  CFloatP controller;
  PFloatTest() : controller(2.0f) {}
};

TEST_F(PFloatTest, ProportionalLimitTest) {
  controller.setLimit(1.5f);
  EXPECT_EQ(0.0f, controller.step(0.0f));
  EXPECT_EQ(1.0f, controller.step(0.5f));
  EXPECT_EQ(1.5f, controller.step(1.0f));
  EXPECT_EQ(-1.5f, controller.step(-2.0f));
}

typedef control::classic::P<int> CIntP;
class PIntTest : public ::testing::Test {
 protected:
  CIntP controller;
  PIntTest() : controller(2) {}
};

TEST_F(PIntTest, ProportionalLimitTest) {
  controller.setLimit(3);
  EXPECT_EQ(0, controller.step(0));
  EXPECT_EQ(-2, controller.step(-1));
  EXPECT_EQ(-3, controller.step(-2));
  EXPECT_EQ(3, controller.step(2));
}

// Ts = 100, K = 2, Ti = 50
typedef control::classic::PI<int> CIntPI;
class PIIntTest : public ::testing::Test {
 protected:
  CIntPI controller;
  PIIntTest() : controller(100, 2, 50) {}
};

TEST_F(PIIntTest, ConstantErrTest) {
  std::vector<int> u(5);
  std::vector<int> v = {4, 8, 12, 16, 20};
  std::generate(u.begin(), u.end(), [this] { return controller.step(1); });

  EXPECT_THAT(u, ::testing::ContainerEq(v));
}

TEST_F(PIIntTest, IntegrlErrTest) {
  // input vector
  std::vector<int> x = {0, 1, 1, -1, -1, 0};
  // validation vector
  std::vector<int> v = {0, 4, 8,  4,  0, 0};
  // test vector
  std::vector<int> u(6);

  // fill test vector with results of input vector
  auto it = x.begin();
  std::generate(u.begin(), u.end(), [this,&it] { return controller.step(*it++); });

  EXPECT_THAT(u, ::testing::ContainerEq(v));
}

}  // namespace
