// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "RobotContainer.h"

#include <rev/CANEncoder.h>
#include <fmt/core.h>
#include <rev/REVLibError.h>
#include <rev/SparkMaxAbsoluteEncoder.h>
#include <rev/SparkMaxAlternateEncoder.h>
#include "rev/SparkMaxRelativeEncoder.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <string>
#include <sstream>
#include <iostream>
#include <ctre/Phoenix.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <frc/Timer.h>
#include <frc/TimedRobot.h>
#include <frc/PowerDistribution.h>
#include <frc/Joystick.h>
#include <frc/BuiltInAccelerometer.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc/AnalogGyro.h>
#include <frc/Solenoid.h>
#include <frc/Compressor.h>
#include <frc/PneumaticsControlModule.h>
#include <frc/PneumaticsModuleType.h>
#include <frc/PneumaticsBase.h>
#include <frc/CompressorConfigType.h>
#include <WPILibVersion.h>
#include <cmath>
#include <math.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/span.h"
#include "cameraserver/CameraServer.h"
#include <chrono>
#include <ctime>
#include <ratio>
#include <string>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/CommandScheduler.h>
#include <ctre/phoenix/motorcontrol/SupplyCurrentLimitConfiguration.h>
#include <rev/REVCommon.h>
#include "rev/CANSparkMax.h"
#include <frc/drive/DifferentialDrive.h>


using namespace std::chrono;

//Joysticks
frc::Joystick JoyStick1(0), Xbox(1), Wheel(2);


//Drivetrain Motors

rev::CANSparkMax FrontLeftMotor {1, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax MiddleLeftMotor {2, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax BackLeftMotor {3, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax FrontRightMotor {4, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax MiddleRightMotor {5, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax BackRightMotor {6, rev::CANSparkMax::MotorType::kBrushless};

// Tank Drive declaration
frc::DifferentialDrive tankDrive{FrontLeftMotor, FrontRightMotor};

//PID Controls

//Intake Motors
rev::CANSparkMax IntakeMotorLeft {7, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax IntakeMotorRight {8, rev::CANSparkMax::MotorType::kBrushless};

// Turntable Motor
rev::CANSparkMax TurnTableMotor {9, rev::CANSparkMax::MotorType::kBrushless};

// Extension of Arm
TalonFX ExtendMotorOne {1};
TalonFX ExtendMotorTwo {2};

// Arm (Up & Down)

rev::CANSparkMax ArmMotorOne {10, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax  ArmMotorTwo {11, rev::CANSparkMax::MotorType::kBrushless};

//You spin me right round baby right round like a record player right round round round
rev::CANSparkMax RotatorMotorOne {12, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax RotatorMotorTwo {13, rev::CANSparkMax::MotorType::kBrushless};

//ClawMotor
rev::CANSparkMax ClawMotor {14, rev::CANSparkMax::MotorType::kBrushless};

//set amp limit
bool enable = true;
double currentLimit = 30;
double triggerThresholdCurrent = 30;
double triggerThresholdTime = .1;

//Arigato Gyro CHU MIN MIN
//Gyro + Accelerometer
WPI_PigeonIMU gyro{0};


//Auto Variables
int autoStep = 1;  

//Timer
steady_clock::time_point clock_begin;

frc::Timer *shooterTimer;
bool timerStarted = false;

bool gyroResetted = false;

void Robot::RobotInit() {

  //camera
  frc::CameraServer::StartAutomaticCapture();

  //Follow front motors
  MiddleLeftMotor.Follow(FrontLeftMotor);
  MiddleRightMotor.Follow(FrontRightMotor);
  BackLeftMotor.Follow(FrontLeftMotor);
  BackRightMotor.Follow(FrontRightMotor);
  ArmMotorTwo.Follow(ArmMotorOne);

  //Neo motor current limit
  FrontLeftMotor.SetSmartCurrentLimit(20);
  MiddleLeftMotor.SetSmartCurrentLimit(20);
  BackLeftMotor.SetSmartCurrentLimit(20);
  FrontRightMotor.SetSmartCurrentLimit(20);
  MiddleRightMotor.SetSmartCurrentLimit(20);
  BackRightMotor.SetSmartCurrentLimit(20);
  ClawMotor.SetSmartCurrentLimit(20);

  // Talon Motor current limit
  SupplyCurrentLimitConfiguration current_limit_config(enable, currentLimit, triggerThresholdCurrent, triggerThresholdTime);
  ExtendMotorOne.ConfigSupplyCurrentLimit(current_limit_config);
  ExtendMotorTwo.ConfigSupplyCurrentLimit(current_limit_config);

  //Setting Idle Mode to brake (neo motors)
  FrontLeftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  MiddleLeftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  BackLeftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  FrontRightMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  MiddleRightMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  BackRightMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  //Idle Mode Claw
  ClawMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

}


void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}


void Robot::AutonomousInit() {
 /* m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  } */
  clock_begin = steady_clock::now();
  
  //Resetting Talons
  ExtendMotorOne.SetSelectedSensorPosition(0);
  ExtendMotorTwo.SetSelectedSensorPosition(0);

  gyro.Reset(); 
}

void Robot::AutonomousPeriodic() {

  // Score Preloaded Cone, then Leaving the community
  if (frc::SmartDashboard::GetNumber("Auto", 1) == 1) {
    if (autoStep == 1) {
      ClawMotor.Set(.50); 
      ExtendMotorOne.SetSelectedSensorPosition(.25);
      ExtendMotorTwo.SetSelectedSensorPosition(.25); 
      autoStep++;
    }
    else if (autoStep == 2) {
      FrontRightMotor.Set(-0.25);
      FrontLeftMotor.Set(-0.25); 
      autoStep++;
    } 
    else if (autoStep == 3) {
      FrontRightMotor.Set(0.25); 
      FrontLeftMotor.Set(0.25); 
      autoStep++; 
    }
  if (frc::SmartDashboard::GetNumber("Auto", 1) == 2){
    if (autoStep == 1) {
      ClawMotor.Set(.50); 
      ExtendMotorOne.SetSelectedSensorPosition(.25);
      ExtendMotorTwo.SetSelectedSensorPosition(.25);
      autoStep++; 
    }
  }
  //Auto for only leaving the community
  if (frc::SmartDashboard::GetNumber("Auto", 1) == 4){
    if (autoStep == 1) {
      FrontRightMotor.Set(-0.25); 
      FrontLeftMotor.Set(-0.25);
      autoStep++; 
    }

  }
 }
}
void Robot::TeleopInit() {

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }
}

void Robot::TeleopPeriodic() {
  //Joystick and Wheel Teleop code
  double WheelX = Wheel.GetX();
  double JoyY = JoyStick1.GetY();
  
  if (JoyStick1.GetRawButton(1) == false) {//new method for driving. Library already has controls for west coast drive
    tankDrive.ArcadeDrive(JoyY, WheelX);
  }
  else if(JoyStick1.GetRawButton(1) == true){//start of point turning
    if (WheelX > 0) {
      FrontRightMotor.Set(-(WheelX * WheelX));
      FrontLeftMotor.Set((WheelX * WheelX));
    } 
    else if (WheelX < 0) {
      FrontRightMotor.Set((WheelX * WheelX));
      FrontLeftMotor.Set(-(WheelX * WheelX));
    }
  }
  else {
    FrontRightMotor.Set(0);
    FrontLeftMotor.Set(0);
  }
}


void Robot::TestPeriodic() {
  while (FrontLeftMotor.Get() <= 2)
    {
      FrontRightMotor.Set(0.1);
      FrontLeftMotor.Set(0.1); 
    }
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
