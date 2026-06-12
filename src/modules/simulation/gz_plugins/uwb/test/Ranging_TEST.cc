#include <gtest/gtest.h>
#include "ranging.pb.h"

TEST(Ranging, FieldsRoundTrip)
{
  px4::msgs::Ranging m;
  m.set_anchor_id(7);
  m.set_tag_id(3);
  m.set_range(1234.5);
  m.set_seq(42);
  m.set_rss(-85.2);
  m.set_error_estimation(0.00393973);
  m.set_angle(1.57);

  EXPECT_EQ(m.anchor_id(), 7);
  EXPECT_EQ(m.tag_id(), 3);
  EXPECT_DOUBLE_EQ(m.range(), 1234.5);
  EXPECT_EQ(m.seq(), 42u);
  EXPECT_DOUBLE_EQ(m.rss(), -85.2);
  EXPECT_DOUBLE_EQ(m.angle(), 1.57);
}
