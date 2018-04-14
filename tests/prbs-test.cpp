#include "control/ident/idsignal.h"
#include "gtest/gtest.h"
#include "gmock/gmock.h"

namespace {

class PRBSTest : public ::testing::Test {
 protected:
  control::ident::PRBS<int> P;
};

TEST_F(PRBSTest, SimplePRBSTest) {
  std::vector<int> r;
  for( int i = 0; i < 1000; i++ )
    r.push_back(P.get());
  EXPECT_THAT(r, ::testing::Each(::testing::AnyOf(-1,1)));
}

class PRBSFloatTest : public ::testing::Test {
 protected:
  control::ident::PRBS<float> P;
};

TEST_F(PRBSFloatTest, SimplePRBSFloatTest) {
  std::vector<float> r;
  for( int i = 0; i < 1000; i++ )
    r.push_back(P.get());
  EXPECT_THAT(r, ::testing::Each(::testing::AnyOf(-1.0,1.0)));
}


}  // namespace
