#include "Movement/Hopper.hpp"
// AKA magazine and the intake
//d pad toggle preset shots 

Hopper::Hopper(
   rev::CANSparkMax &Belt,
   rev::CANSparkMaxLowLevel &Intake,
   FRC5572Controller &Operator,
   frc::DoubleSolenoid &IntakePistions,
   frc::DigitalInput &Input1,
   frc::DigitalInput &Input2,
   frc::DigitalInput &Photo
){
    this->belt = &Belt;
    this->intake = &Intake;
    this->Operator = &Operator;
    this->intakePistions = &IntakePistions;
    this->limitSwitch1 = &Input1;
    this->limitSwitch2 = &Input2;
    this->Photo = &Photo;
}

void Hopper::HopperPeriodic(){
   Hopper::ManualControl();
   Hopper::RunIntakePistions();
   Hopper::Advance();
   Hopper::ManualIntakeMotors();
}

void Hopper::Advance(){
   if( !(Photo->Get()) ){
      if(Operator->X() && !(limitSwitch2->Get()) ){
         belt->Set(.3);
         }
      else {
         belt->Set(0.0);
         }
   }
   /*
   if =(Operator->A()){
      if(!(Photo->Get())&&((limitSwitch1->Get())||(limitSwitch2->Get())){
         belt->Set(0.3);
      }
      if((Photo->Get())){
         belt->Set(0.0);
      }
   }

   */

   if(limitSwitch1->Get()){
      readyToLoad = true;
   }
   else {
      readyToLoad = false;
   }
}

void Hopper::ManualControl(){
   if(this->Operator->R().second > 0.2){
      this->belt->Set(-0.5);
   }
   if(this->Operator->R().second < -0.2){
      this->belt->Set(0.5);
   }
   if(this->Operator->R().second < 0.2 && this->Operator->R().second > -0.2) {
      this->belt->Set(0.0);
   }

}

void Hopper::RunIntakePistions(){
   if(Operator->Y()){
      //intake->Set(.7);
      intakePistions->Set(frc::DoubleSolenoid::Value::kForward); // do toggle
   }
   else{
      //intake->Set(0.0);
      intakePistions->Set(frc::DoubleSolenoid::Value::kReverse);
   }
}

void Hopper::ManualIntakeMotors(){
   if(this->Operator->L().second > 0.2){
      this->intake->Set(-.7);
   }

   if(this->Operator->L().second < -0.2){
      this->intake->Set(.7);
   }
   
   if(this->Operator->L().second < 0.2 && this->Operator->L().second > -0.2) {
      this->intake->Set(0.0);
   }
}