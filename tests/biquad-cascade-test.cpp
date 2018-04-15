#include "control/filter/biquad.h"
#include "gtest/gtest.h"
#include "gmock/gmock.h"

namespace {

using B = control::filter::Biquad<double>;

template<size_t N>
using BC = control::filter::BiquadCascade<B, N>;

class BiquadCascadeTest : public ::testing::Test {
 protected:
  BiquadCascadeTest() : bc({B(1,2,3,4,5), B(1,0,0,0,0)}) {};
  BC<2> bc;
};

TEST_F(BiquadCascadeTest, SimpleTest) {
  std::vector<double> v = { 0, 1, 0, 5, -4 };
  for( int i = 0; i < 5; i++ )
    EXPECT_DOUBLE_EQ( bc.step(i), v[i] );
}

}  // namespace
