#ifndef _XARM_JOINT_H
#define _XARM_JOINT_H

namespace xarm {

class Joint
{
public:
    Joint(int minServo, int maxServo, double minAngle, double maxAngle);

    int getServo();
    void setServo(int servo);
    double getAngle();
    void setAngle(double angle);

    double convertToRadian(int servo);
    int convertToServoReading(double angle);

private:
    int servo = 0;

    // bounds
    int minServo = 0;
    int maxServo = 0;
    double minAngle = 0;
    double maxAngle = 0;
};

} // namespace xarm

#endif
