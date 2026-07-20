/**
 * @file robo-lib.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-07-01
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#ifndef ROBO_LIB_HPP
#define ROBO_LIB_HPP

// algorithm
#include "algorithm/interpolation.hpp"
#include "algorithm/linear-algebra.hpp"
#include "algorithm/matrix.hpp"
#include "algorithm/pid.hpp"
#include "algorithm/quaternion.hpp"
#include "algorithm/rls.hpp"
#include "algorithm/sim-module.hpp"
#include "algorithm/simple-filter.hpp"
#include "algorithm/transform.hpp"

// device
#include "device/device.hpp"
#include "device/dm-motor.hpp"
#include "device/rm-motor.hpp"

// module
#include "module/heat-limit.hpp"
#include "module/vmc-dm.hpp"
#include "module/wheelleg-power-manager.hpp"

// protocol
#include "protocol/fsi6.hpp"
#include "protocol/superpower.hpp"
#include "protocol/upper-lower-comm.hpp"

// utils
#include "utils/convert.hpp"
#include "utils/crc.hpp"
#include "utils/math-adapter.hpp"
#include "utils/math-utils.hpp"

#endif
