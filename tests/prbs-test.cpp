#include "control/ident/idsignal.h"
#include "gtest/gtest.h"
#include "gmock/gmock.h"

namespace {

class PRBSTest : public ::testing::Test {
 protected:
  control::ident::PRBS<int> P;
};

TEST_F(PRBSTest, SimplePRBSTest) {
  std::vector<int> r(1000);
  std::generate(r.begin(), r.end(), [this]{ return P.get(); });
  EXPECT_THAT(r, ::testing::Each(::testing::AnyOf(-1,1)));
}

class PRBSFloatTest : public ::testing::Test {
 protected:
  control::ident::PRBS<float> P;
};

TEST_F(PRBSFloatTest, SimplePRBSFloatTest) {
  std::vector<float> r(1000);
  std::generate(r.begin(), r.end(), [this]{ return P.get(); });
  EXPECT_THAT(r, ::testing::Each(::testing::AnyOf(-1.0,1.0)));
}


}  // namespace
