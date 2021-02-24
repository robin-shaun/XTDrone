/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gtest/gtest.h>
#include "helpers.hh"

/////////////////////////////////////////////////
/// \brief Tests that the basic sandisland world with the wamv was loaded.
class SandislandTest : public ::testing::Test
{
  /// \brief Initialize any members needed for the test cases.
  protected: static void SetUpTestCase();
};

void SandislandTest::SetUpTestCase()
{
}

/////////////////////////////////////////////////
/// \brief Tests that the wamv model exists.
TEST_F(SandislandTest, WamvExists)
{
  EXPECT_TRUE(ModelExists("wamv"));
}

/////////////////////////////////////////////////
/// \brief Tests that the sandisland model exists.
TEST_F(SandislandTest, SandislandExists)
{
  EXPECT_TRUE(ModelExists("sandisland"));
}

/////////////////////////////////////////////////
/// \brief Tests that the ocean model exists.
TEST_F(SandislandTest, OceanExists)
{
  EXPECT_TRUE(ModelExists("ocean_waves"));
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "sandisland_test");
  return RUN_ALL_TESTS();
}
