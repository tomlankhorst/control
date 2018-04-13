#include "control/ident/idsignal.h"
#include "gtest/gtest.h"
#include "gmock/gmock.h"

namespace {

class PrbsTest : public ::testing::Test {
 protected:
  control::ident::prbs P;
};

TEST_F(PrbsTest, SimplePrbsTest) {
  std::vector<int> r;
  for( int i = 0; i < 1000; i++ )
    r.push_back(P.get());
  EXPECT_THAT(r, ::testing::Each(::testing::AnyOf(-1,1)));
}

}  // namespace
