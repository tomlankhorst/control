#include "control/system/statespace.h"
#include "gtest/gtest.h"
#include "gmock/gmock.h"

namespace {

class SecondOrderSISOStatespaceTest : public ::testing::Test {
 protected:
  control::system::Statespace<float, 2> P;
};

TEST_F(SecondOrderSISOStatespaceTest, SimpleTest) {
  P.step({0});
}

}  // namespace
