//
// Created by johannes on 29.04.19.
//

#pragma once
#include <stddef.h>
#include <Eigen/Dense>

namespace smb_mpc
{

  struct SmbDefinitions
  {
    const static size_t STATE_DIM = 7; // postion, orientation quaternion
    const static size_t INPUT_DIM = 2; // linear velocity, angular velocity
    const static size_t REFERENCE_STATE_DIM_ =
        7; // postion, orientation quaternion

    static Eigen::VectorXd defaultState();
  };

}
