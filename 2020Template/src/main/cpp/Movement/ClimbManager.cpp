#include "Movement/ClimbManager.hpp"

#include <frc/DoubleSolenoid.h>
ClimbManager::ClimbManager(
    frc::VictorSP& LeftClimb,
    frc::VictorSP& RightClimb,
    FRC5572Controller& Driver,
    frc::DoubleSolenoid& ClimbPistons
){
    this->leftClimb = &LeftClimb;
    this->rightClimb = &RightClimb;
    this->driver = &Driver;
    this->climbPistons = &ClimbPistons;
    //this->climbMotors = new frc::SpeedControllerGroup(LeftClimb, RightClimb);
    //this->leftClimb->SetInverted(true);
    //this->rightClimb->SetInverted(false);

}

void ClimbManager::ClimbPeriodic(){
    ClimbManager::Spin();
    ClimbManager::UpAndDown();
}

void ClimbManager::UpAndDown(){
    if(driver->LT() > 0){
        this->climbPistons->Set(frc::DoubleSolenoid::Value::kReverse); 
    }
    else if(driver->RT() > 0){
       this->climbPistons->Set(frc::DoubleSolenoid::Value::kForward);
    }
    else{
        this->climbPistons->Set(frc::DoubleSolenoid::Value::kOff);
    }
    
}

void ClimbManager::Down(){
    
}

void ClimbManager::Spin(){

    if(driver->RB()){
        //this->leftClimb->Set(-0.3);
        //this->climbMotors->Set(-0.3);
        leftClimb->Set(.4);
        rightClimb->Set(.4);
    }
    else if(driver->LB()){
        //this->leftClimb->Set(0);
        //this->climbMotors->Set(0.0); / problem 
        leftClimb->Set(-0.4);
        rightClimb->Set(-0.4);
    }
    else{
        leftClimb->Set(0.0);
        rightClimb->Set(0.0);
    }
}
