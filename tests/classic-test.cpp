#include "control/classic.h"
#include "gtest/gtest.h"
#include "gmock/gmock.h"

namespace {

typedef control::classic::P<double> CDoubleP;
typedef control::classic::PI<double> CDoublePI;
class PDoubleTest : public ::testing::Test {
 protected:
  CDoubleP controller;
  PDoubleTest() : controller(2.0) {}
};

// PI controller K=2, Ti=1, Ts=0.1
class PIDoubleTest : public ::testing::Test {
 protected:
  CDoublePI controller;
  PIDoubleTest() : controller(0.1, 2.0, 1.0) {}
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

TEST_F(PIDoubleTest, LimitErrTest) {
  std::vector<double> u;
  std::vector<double> v = {2.1, 2.2, 2.3, 2.3, 2.3};

  controller.setLimit(2.3);

  // 10 steps
  for(int i = 0; i < 5; i++)
    u.push_back(controller.step(1.0));

  EXPECT_THAT(u, ::testing::ContainerEq(v));
}


TEST_F(PIDoubleTest, ConstantErrTest) {
  std::vector<double> u;
  std::vector<double> v = {2.1, 2.2, 2.3, 2.4, 2.5};

  // 10 steps
  for(int i = 0; i < 5; i++)
    u.push_back(controller.step(1.0));

  EXPECT_THAT(u, ::testing::ContainerEq(v));
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

}  // namespace
