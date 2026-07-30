#include "arm.h"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <thread>
#include <valarray>

void Arm::init() {
  for (const auto &motor : motors) {
    // motor->setGoalVelocity(400);
    // motor->setVelocityLimit(100);
    // motor->setAccelerationLimit(30);
    if (motor->readHardwareErrorStatus() != 0) {
      motor->reboot();
    }

    motor->setProfileVelocity(0);
    motor->setProfileAcceleration(0);
    motor->setTorqueEnable(Torque::ENABLE);
  }
  // base.setPositionPGain(400);
  // shoulder.setPositionPGain(400);

  // wrist.setGoalVelocity(1000);
  // pitchWrist.setProfileVelocity(1800);
  // pitchWrist.setProfileAcceleration(450);

  std::thread([&] {
    resetByZ(50);
    resetted = false;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    resetByZ(400);
    resetted = false;
  }).detach();
}

bool Arm::inverseKinematics(const double x, const double y, const double z,
                            double &q1, double &q2, double &q3, double &q4,
                            const double pitch, const double yaw) {
  q2 = 0;

  const auto l1 = 223.602;
  const auto l2 = 151.80;
  const auto l3 = 103.333;

  const auto x2 = x - l3 * std::cos(pitch);
  const auto z2 = z - l3 * std::sin(pitch);

  // 2R solve
  const auto r_square = x2 * x2 + z2 * z2;
  const auto cos_theta2 = (r_square - l1 * l1 - l2 * l2) / (2 * l1 * l2);
  const auto sin_theta2 = std::sqrt(1 - cos_theta2 * cos_theta2);

  const auto theta2 = std::atan2(sin_theta2, cos_theta2);
  const auto theta1 =
      std::atan2(z2, x2) - std::atan2(l2 * sin_theta2, l1 + l2 * cos_theta2);
  const auto theta3 = pitch - theta1 - theta2;

  if (std::isnan(theta1) || std::isnan(theta2) || std::isnan(theta3)) {
    return false;
  }
  q1 = 180 + theta1 / M_PI * 180;
  q2 = 0;
  q3 = 180 + theta2 / M_PI * 180;
  q4 = 180 + theta3 / M_PI * 180;
  return true;
}

void Arm::move(const double y, const double z, const bool hitTarget) {
  std::thread([&, y, z, hitTarget] {
    if (!mtx.try_lock()) {
      return;
    }
    try {
      for (;;) {
        double q1, q2, q3, q4;
        int maxX = 320;
        for (; maxX > 120 &&
               !inverseKinematics(hitTarget ? maxX : 120, 0,
                                  z + (hitTarget ? 40 : 0), q1, q2, q3, q4,
                                  (hitTarget ? 60 : 100) * M_PI / 180);
             --maxX)
          ;
        if (maxX <= 120) {
          break;
        }
        auto writer = shoulderPitch.getBulkWriter();
        shoulderPitch.setAngleBulk(writer, q1);
        shoulderPitchRev.setAngleBulk(writer, -q1);
        shoulderYaw.setAngleBulk(writer, q2);
        elbow.setAngleBulk(writer, q3);
        wrist.setAngleBulk(writer, q4);
        if (const int result = writer.txPacket(); result != COMM_SUCCESS) {
          std::cerr << dynamixel::PacketHandler::getPacketHandler()
                           ->getTxRxResult(result)
                    << std::endl;
          throw std::runtime_error("Failed to send bulk write packet");
        }
        break;
      }
    } catch (const std::exception &e) {
      std::cerr << e.what() << std::endl;
    }
    resetted = false;
    mtx.unlock();
  }).detach();
}

void Arm::resetByZ(const double z) {
  if (resetted)
    return;
  std::thread([&, z] {
    if (!mtx.try_lock()) {
      return;
    }
    try {
      for (;;) {
        double q1, q2, q3, q4;
        if (!inverseKinematics(120, 0, z, q1, q2, q3, q4)) {
          break;
        }
        auto writer = shoulderPitch.getBulkWriter();
        shoulderPitch.setAngleBulk(writer, q1);
        shoulderPitchRev.setAngleBulk(writer, -q1);
        shoulderYaw.setAngleBulk(writer, q2);
        elbow.setAngleBulk(writer, q3);
        wrist.setAngleBulk(writer, q4);
        resetted = true;
        break;
      }
    } catch (const std::exception &e) {
      std::cerr << e.what() << std::endl;
    }
    mtx.unlock();
  }).detach();
}
