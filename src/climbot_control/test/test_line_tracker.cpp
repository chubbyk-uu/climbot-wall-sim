#include "climbot_control/line_tracker.hpp"
#include "gtest/gtest.h"
using namespace climbot_control;
TEST(LineTracker, CorrectsDownwardCrossTrackWithUpwardHeading) {auto c=trackLine({0,0},{1,0},{0, -0.1, 0},.15,1,2,{}); EXPECT_GT(c.angular,0); EXPECT_NEAR(c.cross,-.1,1e-9);}
TEST(LineTracker, WorksForVerticalAndDiagonalLines) {EXPECT_NEAR(trackLine({0,0},{0,1},{.05,.4,1.57079632679},.15,1,2,{}).cross,-.05,1e-9); EXPECT_GT(trackLine({0,0},{1,1},{.5,.5,0},.15,1,2,{}).angular,0);}
TEST(LineTracker, JointWheelSaturationPreservesCurvature) {Command d{.3,1.2}; auto c=rateLimit(d,{},1,1,2,.43,.3); EXPECT_LE(std::abs(c.linear+c.angular*.43/2),.3+1e-9); EXPECT_NEAR(c.angular/c.linear,d.angular/d.linear,1e-9);}
