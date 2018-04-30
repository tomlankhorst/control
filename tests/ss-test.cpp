#include "control/system/ss.h"
#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include <Eigen/Dense>
#include <iostream>

/**
 * Discrete time state-space tests
 * @todo write tests for MIMO
 */
namespace {

class SSTest : public ::testing::Test {
public:
    using ss = control::system::SS<float,2>;
protected:
  ss *P; 
  SSTest() {
    ss::TA A;
    ss::TB B;
    ss::TC C;
    ss::TD D;
    A << 1, 1, 0, 1;
    B << 0.5, 1;
    C << 1, 0;
    D << 0;
    P = new ss(A,B,C,D);      
  }
};

TEST_F(SSTest, SimpleSSTest) {
  std::vector<float> r(10);
  std::vector<float> v = {0.5, 2, 4.5, 8, 12.5, 18, 24.5, 32, 40.5};
  std::generate(r.begin(), r.end(), [this]{ 
                ss::Tu u;
                u << 1;
                auto y = P->step(u); 
                return y(0);
                });

	for( size_t i = 0; i < v.size(); i++ ) 
		ASSERT_FLOAT_EQ(r[i], v[i]);

}

}  // namespace
