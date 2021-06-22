//
// Created by johannes on 02.05.19.
//
#include <gtest/gtest.h>
#include <smb_mpc/SmbInterface.h>
#include <cmath>

using namespace smb_path_following;

TEST(SmbInterfaceTests, instantiation) {
  SmbInterface mmInterface;

  ASSERT_TRUE(true);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
