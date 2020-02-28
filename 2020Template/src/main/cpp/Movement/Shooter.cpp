#include "Movement/Shooter.hpp"


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

Shooter::Shooter(
    rev::CANSparkMax &LeftMotor,
    rev::CANSparkMax &RightMotor,
    frc::DoubleSolenoid &Hood,
    FRC5572Controller &Operator
    ){
    this->leftMotor = &LeftMotor;
    this->rightMotor = &RightMotor;
    this->Hood = &Hood;
    this->Operator = &Operator;
    //this->leftMotor->SetInverted(true);
    //this->rightMotor->SetInverted(true);
    shooterMotors = new frc::SpeedControllerGroup{ LeftMotor, RightMotor};

}

void Shooter::Shot(){
    if(Operator->B()){
        Hood->Set(frc::DoubleSolenoid::Value::kForward); //do toggle
    }
    else{
      Hood->Set(frc::DoubleSolenoid::Value::kReverse);  
    }

    if(Tracked)
    {
        shooterMotors->Set(Operator->RT());
    }
    else
    {
        shooterMotors->Set(0); 
    }
}

void Shooter::Calucate(){
    
}

void Shooter::TestRPM(){

}

void Shooter::BumberShot(){
    /*
    if(Operator->BLaBla){
        shooterMotors->Set(1.0);
    }
    */
}
/*
void Shooter::InitShot
*/