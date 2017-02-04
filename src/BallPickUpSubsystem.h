#ifndef BALLPICKUPSUBSYSTEM_H_
#define BALLPICKUPSUBSYSTEM_H_

#include <WPILib.h>
#include <CANTalon.h>
#include <RobotSubsystem.h>

class BallPickUpSubsystem: public RobotSubsystem {
public:
    BallPickUpSubsystem(EntechRobot *pRobot, std::string name);
    virtual ~BallPickUpSubsystem();
    
    void Off(void);
    void Forward(void);
    void Backward(void);
    void BallOutForKicker();
    void BallOutForShooter();
    void BallInToHold();
    void Abort();
    bool Done();

    /********************************** Init Routines *************************************/

    virtual void RobotInit();
    virtual void DisabledInit();
    virtual void TeleopInit();
    virtual void AutonomousInit();
    virtual void TestInit();

    /********************************** Periodic Routines *************************************/

    virtual void DisabledPeriodic();
    virtual void AutonomousPeriodic();
    virtual void TeleopPeriodic();
    virtual void TestPeriodic();

private:
    CANTalon *m_motor;
    double m_speed;
    DigitalInput* m_pickupSensor;
    Timer *m_timer;
    double m_timeout;
    enum BallPickupState { kNormal, kOutToSensor, kOutForKicker, kOutForShooter, kInToHold, kHolding };
    BallPickupState m_state;
    bool m_ballSeen;
};

#endif /* BALLPICKUPSUBSYSTEM_H_ */
