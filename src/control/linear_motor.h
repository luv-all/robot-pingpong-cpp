#ifndef LINEAR_MOTOR_H
#define LINEAR_MOTOR_H

class LinearMotor {
  int axisNo;
  static bool isMotionModule();
  double min, max;
  [[nodiscard]] double getClampedPosition(double position) const;
  double targetPosition;

public:
  LinearMotor() = delete;
  explicit LinearMotor(int axisNo);

  [[nodiscard]] bool hasLimit() const;
  [[nodiscard]] double map(double value, double min, double max) const;
  [[nodiscard]] double getPosition() const;
  [[nodiscard]] double getMappedPosition(double min, double max) const;
  void setPosition(double position, bool wait = true);
  [[nodiscard]] bool hasAlarm() const;
  void resetAlarm() const;
  void setMaxVelocity(double velocity) const;
  void update();
  [[nodiscard]] bool isMoving() const;
  void off() const;
  void on() const;

  ~LinearMotor();
};

#endif // LINEAR_MOTOR_H
