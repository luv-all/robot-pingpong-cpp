#ifndef ARM_H
#define ARM_H

#include "../constants.h"
#include "../dynamixel/mx28_p2.h"
#include "../dynamixel/mx64_p2.h"

#include <cmath>
#include <list>
#include <mutex>

class Arm {
  Servos::Mx64P2 shoulderPitch = Servos::Mx64P2(PORT_NAME, SHOULDER_ID);
  Servos::Mx64P2 shoulderPitchRev =
      Servos::Mx64P2(PORT_NAME, SHOULDER_PRIME_ID);
  Servos::Mx64P2 shoulderYaw = Servos::Mx64P2(PORT_NAME, SHOULDER_YAW_ID);
  Servos::Mx28P2 elbow = Servos::Mx28P2(PORT_NAME, ELBOW_ID);
  Servos::Mx28P2 wrist = Servos::Mx28P2(PORT_NAME, WRIST_ID);
  std::list<BaseMotor *> motors = {&shoulderPitch, &shoulderPitchRev,
                                   &shoulderYaw, &elbow, &wrist};
  bool resetted = false;
  std::mutex mtx;
  static bool inverseKinematics(double x, double y, double z, double &q1,
                                double &q2, double &q3, double &q4,
                                double pi = M_PI / 2, double yawPi = 0);

public:
  void init();
  void move(double y, double z, bool hitTarget);
  void resetByZ(double z);
};

#endif // ARM_H
