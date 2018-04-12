#include "control/filter/biquad.h"
#include "gtest/gtest.h"
#include "gmock/gmock.h"

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

TEST_F(BiquadTest, InfBiquadTest) {
  double Inf = std::numeric_limits<double>::infinity();
  double Nan = NAN;
  std::vector<double> x = { 0, 0, 0, 0, Inf };
  std::vector<double> v = { 0, 0, 0, 0, Inf };
  for( int i = 0; i < 5; i++ )
    EXPECT_DOUBLE_EQ( b.step(x[i]), v[i] );
}

}  // namespace
