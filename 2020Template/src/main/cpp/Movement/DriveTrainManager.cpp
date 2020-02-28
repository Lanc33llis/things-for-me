#include "Movement/DriveTrainManager.hpp"

/* CAN ID layout for drive train from a top view

          Front of Robot
         
         |--------------|
         |              |
      1  | M1        M2 | 2
      3  | M3        M4 | 3
      5  | M5        M6 | 6
         |              |
         |              |
         |--------------|

          Back Of Robot
*/

DriveTrain::DriveTrain(
    rev::CANSparkMax &TopLeftMotor,
    rev::CANSparkMax &TopRightMotor,

    rev::CANSparkMax &MiddleLeft,
    rev::CANSparkMax &MiddleRight,

    rev::CANSparkMax &BottomLeftMotor, 
    rev::CANSparkMax &BottomRightMotor,

    FRC5572Controller &Driver,
    VisionManager &VisionManager,
    AHRS &ahrs
    ){
        this->LeftMotors = new frc::SpeedControllerGroup( TopLeftMotor, MiddleLeft, BottomLeftMotor);
        this->RightMotors = new frc::SpeedControllerGroup( TopRightMotor, MiddleRight, BottomRightMotor);
        
        this->Driver = &Driver;
        this->ahrs = &ahrs;

        this->TopLeftMotor = &TopLeftMotor;
        this->TopRightMotor = &TopRightMotor;

        this->MiddleLeft = &MiddleLeft;
        this->MiddleRight = &MiddleRight;

        this->BottomLeftMotor = &BottomLeftMotor;
        this->BottomRightMotor = &BottomRightMotor;

        this->TopLeftMotorEncoder = new rev::CANEncoder{TopLeftMotor};
        this->TopRightMotorEncoder = new rev::CANEncoder{TopRightMotor};

        this->MiddleLeftMotorEncoder = new rev::CANEncoder{MiddleLeft};
        this->MiddleRightMotorEncoder = new rev::CANEncoder{MiddleRight};
        
        this->BottomLeftMotorEncoder = new rev::CANEncoder{BottomLeftMotor};
        this->BottomRightMotorEncoder = new rev::CANEncoder{BottomRightMotor};

        //this->LeftMotors->SetInverted(true);

        //this->TopleftMotor->SetInverted(true);

        this->LimeLight = &VisionManager;
}

DriveTrain::~DriveTrain()
{
    delete LeftMotors;
    delete RightMotors;
    delete Driver;
    delete ahrs;
}

void DriveTrain::Drive()
{
    //DriveTrain::Aim();
    LeftMotors->Set(-1 * Driver->L().second );
    RightMotors->Set(Driver->R().second );

}

void DriveTrain::LowerAmps(){
    TopLeftMotor->SetSmartCurrentLimit(40);
    TopRightMotor->SetSmartCurrentLimit(40);
    BottomLeftMotor->SetSmartCurrentLimit(40);
    BottomRightMotor->SetSmartCurrentLimit(40);
}

void DriveTrain::Aim(){
    std::cout << "asd" << std::endl; 
    disX = LimeLight->disX; 
    std::cout << "disX is: " << disX << std::endl; 
    if (Driver->A())
    {
    if (fabs(disX) > 0)
    {
        if (fabs(disX) > 10)
        {
            copysign(1, disX) == 1 ? RightMotors->Set(.3) : LeftMotors->Set(.3);
        }
        if (fabs(disX) < 10)
        {
            copysign(1, disX) == 1 ? RightMotors->Set(disX/30) : LeftMotors->Set(disX/30);
        }
    }
    else if (Driver->A() == false)
    {
        RightMotors->Set(0); LeftMotors->Set(0);
    }
    }
}
