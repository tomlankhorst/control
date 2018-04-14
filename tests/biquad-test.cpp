#include "control/filter/biquad.h"
#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include <cmath>

namespace {

typedef control::filter::Biquad<double> B;

class BiquadTest : public ::testing::Test {
 protected:
  B b;
  BiquadTest() : b(1, 2, 3, 1, 2) {}
};

TEST_F(BiquadTest, SimpleBiquadTest) {
  std::vector<double> v = { 0, 1, 3, 5, 5 };
  for( int i = 0; i < 5; i++ )
    EXPECT_DOUBLE_EQ( b.step(i), v[i] );
}

TEST_F(BiquadTest, StabilityTest) {
  EXPECT_FALSE(b.stable());
}

TEST_F(BiquadTest, InfBiquadTest) {
  double Inf = std::numeric_limits<double>::infinity();
  double Nan = NAN;

  std::vector<double> x = { 0, 0, 0, 0, Inf };
  std::vector<double> v = { 0, 0, 0, 0, Inf };
  for( int i = 0; i < 5; i++ )
    EXPECT_DOUBLE_EQ( b.step(x[i]), v[i] );

}

/**
 * Marginally stable
 *
 * 0.5000 + 0.8660i (abs 1)
 * 0.5000 - 0.8660i (abs 1)
 */
class MarginallyStableTest : public ::testing::Test {
 protected:
  B b;
  MarginallyStableTest() : b(0, 1, -1, -1, 1) {}
};

/**
 * Test that the biquad stays bounded with a marginally stable configration
 */
TEST_F(MarginallyStableTest, StaysBounded){

  b.step(1);

  // Ensure that in 100 samples, value within 1
  for( int i = 0; i < 100; i++ )
    ASSERT_LE(std::abs(b.step(0)),1);

}

/**
 * Determine the poles of the system
 */
TEST_F(MarginallyStableTest, PoleTest){
  auto p = b.poles();
  auto p1 = std::get<0>(p), p2 = std::get<1>(p);

  EXPECT_DOUBLE_EQ((double)abs(p1), 1.0);
  EXPECT_DOUBLE_EQ((double)abs(p2), 1.0);
  EXPECT_TRUE(b.stable());
}

}  // namespace
