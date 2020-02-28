#ifndef SHOOTER_HPP
#define SHOOTER_HPP

#include <frc/DoubleSolenoid.h>
#include "rev/CANSparkMax.h"
#include "AHRS.h"
#include "Movement/ControllerManager.hpp"
#include <frc/SpeedControllerGroup.h>

/* CAN ID layout for drive train from a top view

          Front of Robot
         
         |--------------|
         |              |
      ?  |    M?   M?   | ?
         |              | 
         |              | 
         |              |
         |              |
         |--------------|

          Back Of Robot
*/

class Shooter{
    public:

    Shooter(
        rev::CANSparkMax &LeftMotor,
        rev::CANSparkMax &RightMotor,
        frc::DoubleSolenoid &Hood,
        FRC5572Controller &Operator
    );

    void Shot();
    void Calucate();
    void TestRPM();

    void BumberShot();
    
    bool Tracked = true, hood = false;
    double Power = 0, Distance = 0;


    

    /* Notes of power percentage and RPMs to make shot at certain distances*/
    // Bumber shot = %- RPM - 
    // Initiation line = %- RPM - 
    // Trench = %- RPM - 
    // Color Wheel = %- RMP - 
    frc::SpeedControllerGroup* shooterMotors;
    
    FRC5572Controller* Operator;

    rev::CANSparkMax* leftMotor;
    rev::CANSparkMax* rightMotor;

    frc::DoubleSolenoid* Hood;
};

#endif