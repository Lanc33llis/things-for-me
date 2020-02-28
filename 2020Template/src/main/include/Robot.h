#pragma once

#include <string>

#include "Vision/ColorManager.h"
#include "Vision/VisionManager.hpp"
#include "Movement/ControllerManager.hpp"
#include "Movement/DriveTrainManager.hpp"
#include "Movement/Shooter.hpp"
#include "Movement/Hopper.hpp"
#include "Movement/ClimbManager.hpp"
#include "vision/PhotoeletricSensor.hpp"
#include "Auto/Auto.h"

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include <frc/util/color.h>
#include <frc/DigitalInput.h>

#include "AHRS.h"
#include "rev/CANSparkMax.h"
#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>
#include <frc/VictorSP.h>

class Robot : public frc::TimedRobot
{
 private:
 VisionManager LimeLight;

 AutoMovement *automovement;
 Timer timer;
 // Controllers
  FRC5572Controller Driver{0}; 
  FRC5572Controller Operator{1};

  //Nav-XMP board
  AHRS ahrs{SPI::Port::kMXP};

  /* DriveTrain Spark Max and Motors*/
  rev::CANSparkMax m_leftTopMotor{TopLeft, 
    rev::CANSparkMax::MotorType::kBrushless};

  rev::CANSparkMax m_rightTopMotor{TopRight, 
    rev::CANSparkMax::MotorType::kBrushless};

  rev::CANSparkMax m_leftMiddleMotor{MiddleLeft, 
    rev::CANSparkMax::MotorType::kBrushless};

  rev::CANSparkMax m_rightMiddleMotor{RightMid, 
    rev::CANSparkMax::MotorType::kBrushless};

  rev::CANSparkMax m_leftBottomMotor{LeftMid, 
    rev::CANSparkMax::MotorType::kBrushless};

  rev::CANSparkMax m_rightBottomMotor{RightMid, 
    rev::CANSparkMax::MotorType::kBrushless};
 
 /* Shooters Spark Max and Motors*/
  rev::CANSparkMax m_leftShooter{LeftShoot, 
    rev::CANSparkMax::MotorType::kBrushless};

  rev::CANSparkMax m_rightShooter{RightShoot, 
    rev::CANSparkMax::MotorType::kBrushless};
 
 /* Intake */
  rev::CANSparkMax m_intake{Intake, 
    rev::CANSparkMax::MotorType::kBrushless};

/* Hopper */
  rev::CANSparkMax m_hopper{HopperID, 
    rev::CANSparkMax::MotorType::kBrushless};

/* Climber */
  // rev::CANSparkMax m_LeftClimb{LeftClimb, 
  //   rev::CANSparkMax::MotorType::kBrushed};

  // rev::CANSparkMax m_RightClimb{RightClimb, 
  //   rev::CANSparkMax::MotorType::kBrushed};

  
  frc::VictorSP test1 {0}; // left climb motor

  frc::VictorSP test2 {1}; // right climb motor

  /*instantiation of the compressor with its CAN ID and pneumatics*/ 
  Compressor compressor{PCM1};

  frc::DoubleSolenoid climb{PCM1, 1, 6}; // 3 4

  frc::DoubleSolenoid shooterHood{PCM1, 2, 5}; 

  frc::DoubleSolenoid intake{PCM1 ,3 ,4};  //1 6

  // Sensor
  frc::DigitalInput limitSwitch1{3};
  frc::DigitalInput limitSwitch2{2};

  frc::DigitalInput photoIN{0};
  frc::DigitalOutput photoOUT{1};

  /*SubSystem Objects  */
  DriveTrain driveTrain{ m_leftTopMotor, m_rightTopMotor, m_leftMiddleMotor, m_rightMiddleMotor, m_leftBottomMotor, m_rightBottomMotor, Driver, LimeLight, ahrs};

  Shooter shooter{m_leftShooter, m_rightShooter, shooterHood, Operator};
  
  ClimbManager climber{test1, test2, Driver, climb};

  Hopper hopper{m_hopper, m_intake, Operator, intake, limitSwitch1, limitSwitch2, photoIN};  
  
  Photoelctric photoSensor{photoIN, photoOUT};

  /* IDS */
  static const int
  TopLeft = 1, // GOOD
  TopRight = 2, // GOOD
  
  MiddleLeft = 3, // GOOD
  MiddleRight = 4, // GOOD

  LeftMid = 5, // GOOD
  RightMid = 6, // GOOD

  LeftShoot = 7, // GOOD
  RightShoot = 8, // GOOD

  Intake = 9, // GOOD
  
  PCM1 = 10, // GOOD

  HopperID = 11, //GOOD
  
  LeftClimb = 13, //GOOD
  RightClimb = 14; //GOOD

 public:
  void RobotInit() override;

  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
};





// ¶¶¶¶¶¶¶¶¶¶´¶¶¶¶¶¶¶
// ¶¶¶¶¶¶¶¶__________¶¶ 
// ¶¶¶¶¶_______________¶¶¶
// ¶¶¶____________________¶
// ¶¶______¶¶______________¶
// ¶______¶¶¶¶______________¶
// ¶______¶¶¶¶____¶¶¶¶______¶
// ¶_______¶¶_______________¶
// ¶________________________¶
// ¶_____¶¶__________¶¶_____¶
// ¶¶_____¶¶________¶¶_____¶
// ¶¶¶______¶¶¶¶¶¶¶¶¶_____¶
// ¶¶¶¶¶________________¶¶
// ¶¶¶¶¶¶¶¶___________¶¶
// ¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶
