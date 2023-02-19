// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Libraries
#include "Robot.h"
#include "RobotContainer.h"
#include "cameraserver/CameraServer.h"

#include <rev/CANEncoder.h>
#include <fmt/core.h>
#include <rev/REVLibError.h>
#include <rev/SparkMaxAbsoluteEncoder.h>
#include <rev/SparkMaxAlternateEncoder.h>
#include "rev/SparkMaxRelativeEncoder.h"
#include <string>
#include <sstream>
#include <iostream>

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
#include "cameraserver/CameraServer.h"
#include <chrono>
#include <ctime>
#include <ratio>
#include <string>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/CommandScheduler.h>

#include <rev/REVCommon.h>
#include "rev/CANSparkMax.h"
#include <frc/drive/DifferentialDrive.h>
#include <cstdio>
#include <thread>
#include <rev/CANPIDController.h>
#include <rev/CANSensor.h>
#include <rev/CANDeviceScanner.h>
#include <frc/apriltag/AprilTagDetection.h>
#include <frc/apriltag/AprilTagDetector.h>
#include <frc/apriltag/AprilTagPoseEstimator.h>
#include <frc/ComputerVisionUtil.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <units/angle.h>
#include <units/length.h>

/*#include <ctre/phoenix/motorcontrol/SupplyCurrentLimitConfiguration.h>
#include <ctre/Phoenix.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h> */

using namespace std::chrono;

// Joysticks
frc::Joystick JoyStick1(0), Xbox(2), Wheel(1);

// Drivetrain Motors

rev::CANSparkMax FrontLeftMotor{1, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax MiddleLeftMotor{3, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax BackLeftMotor{5, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax FrontRightMotor{2, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax MiddleRightMotor{4, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax BackRightMotor{6, rev::CANSparkMax::MotorType::kBrushless};



//Drivetrain encoders


rev::SparkMaxRelativeEncoder LeftEncoder = FrontLeftMotor.GetEncoder();
rev::SparkMaxRelativeEncoder RightEncoder = FrontRightMotor.GetEncoder();




rev::CANSparkMax ArmUpOne{7, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax ArmUpTwo{8, rev::CANSparkMax::MotorType::kBrushless};
rev::SparkMaxRelativeEncoder ArmEncoder = ArmUpOne.GetEncoder();


rev::CANSparkMax TurnTable{9, rev::CANSparkMax::MotorType::kBrushless};

rev::CANSparkMax ExtentionMotor{10, rev::CANSparkMax::MotorType::kBrushless};



double LeftEncoderValue;
double RightEncoderValue;


/*
// PID Controls

*/
// set amp limit
bool enable = true;
double currentLimit = 30;
double triggerThresholdCurrent = 30;
double triggerThresholdTime = .1;

// Arigato Gyro CHU MIN MIN
// Gyro + Accelerometer
//WPI_PigeonIMU gyro{0};

// Auto Variables
int autoStep = 1;

// Timer
steady_clock::time_point clock_begin;

frc::Timer *shooterTimer;
bool timerStarted = false;

bool gyroResetted = false;

bool buttonValue;
bool buttonValueTwo;
bool buttonValueThree;
bool buttonValueFour;

float AverageEncoderValue;
void Robot::RobotInit()
{

  RightEncoder.SetPosition(0);
  LeftEncoder.SetPosition(0);
  // camera
  frc::CameraServer::StartAutomaticCapture();

  // Follow ExtendMotorOne
  //ExtendMotorTwo.Follow(ExtendMotorOne);

  // Follow front motors
  MiddleLeftMotor.Follow(FrontLeftMotor);
  BackLeftMotor.Follow(FrontLeftMotor);
  MiddleRightMotor.Follow(FrontRightMotor);
  BackRightMotor.Follow(FrontRightMotor);
  ArmUpTwo.Follow(ArmUpOne);
 // ArmMotorTwo.Follow(ArmMotorOne);

  // Neo motor current limit
  FrontLeftMotor.SetSmartCurrentLimit(40);
  MiddleLeftMotor.SetSmartCurrentLimit(40);
  BackLeftMotor.SetSmartCurrentLimit(40);
  FrontRightMotor.SetSmartCurrentLimit(40);
  MiddleRightMotor.SetSmartCurrentLimit(40);
  BackRightMotor.SetSmartCurrentLimit(40);
  //ClawMotor.SetSmartCurrentLimit(40);

  // Talon Motor current limit
  //SupplyCurrentLimitConfiguration current_limit_config(enable, currentLimit, triggerThresholdCurrent, triggerThresholdTime);
  //ExtendMotorOne.ConfigSupplyCurrentLimit(current_limit_config);
  //ExtendMotorTwo.ConfigSupplyCurrentLimit(current_limit_config);

  // Setting Idle Mode to brake (neo motors)
  FrontLeftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  MiddleLeftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  BackLeftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  FrontRightMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  MiddleRightMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  BackRightMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  float AverageEncoderValue;
 
  

  // Idle Mode Claw
 // ClawMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  // Auto Chooser

  /*
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
 */
}

void Robot::RobotPeriodic()
{
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousInit()
{

  autoStep = 1;
  RightEncoder.SetPosition(0);
  LeftEncoder.SetPosition(0);
  /* m_autonomousCommand = m_container.GetAutonomousCommand();

   if (m_autonomousCommand != nullptr) {
     m_autonomousCommand->Schedule();
   } */
  clock_begin = steady_clock::now();

  frc::CameraServer::StartAutomaticCapture();

  // Resetting Talons
  //ExtendMotorOne.SetSelectedSensorPosition(0);
  //ExtendMotorTwo.SetSelectedSensorPosition(0);

  //gyro.Reset();
  /*
    frc::AprilTagPoseEstimator::Config poseEstConfig = {
          .tagSize = units::length::inch_t(6.0),
          .fx = 699.3778103158814,
          .fy = 677.7161226393544,
          .cx = 345.6059345433618,
          .cy = 207.12741326228522};
    frc::AprilTagPoseEstimator estimator =
          frc::AprilTagPoseEstimator(poseEstConfig);

    cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();
    camera.SetResolution(640, 480);
    cs::CvSink cvSink = frc::CameraServer::GetVideo();
    cs::CvSource outputStream =
          frc::CameraServer::PutVideo("Detected", 640, 480);
    std::vector<int> tags;
      cv::Scalar outlineColor = cv::Scalar(0, 255, 0);
      cv::Scalar crossColor = cv::Scalar(0, 0, 255);
    cv::cvtColor(mat, grayMat, cv::COLOR_BGR2GRAY);

        cv::Size g_size = grayMat.size();
        frc::AprilTagDetector::Results detections =
            detector.Detect(g_size.width, g_size.height, grayMat.data);

        // have not seen any tags yet
        tags.clear();*/
}

void Robot::AutonomousPeriodic()
{

  std::string autoChooser = frc::SmartDashboard::GetString("DB/String 2", "myDefaultData");
  if(autoChooser == "1"){
    frc::SmartDashboard::PutString("DB/String 3", "AutoOne");
  }
  else if(autoChooser == "2"){
    frc::SmartDashboard::PutString("DB/String 3", "AutoTwo");
  } 

AverageEncoderValue = (LeftEncoderValue + RightEncoderValue)/2;

LeftEncoderValue = -LeftEncoder.GetPosition();
RightEncoderValue = RightEncoder.GetPosition();


frc::SmartDashboard::PutString("DB/String 0", ("Left: " + std::to_string(LeftEncoderValue)));
frc::SmartDashboard::PutString("DB/String 1", ("Right: " + std::to_string(RightEncoderValue)));

/*if (LeftEncoderValue <= 4 || RightEncoderValue <= 4) {
  FrontLeftMotor.Set(-0.1);
  FrontRightMotor.Set(0.1);
}
else {
  FrontLeftMotor.Set(0);
  FrontRightMotor.Set(0);
}*/

if (autoChooser == "1") {
  if (autoStep == 1 && AverageEncoderValue >= -20) {
    FrontRightMotor.Set(-0.20);
    FrontLeftMotor.Set(0.20);

    autoStep++;

  }
  else if (autoStep == 2 && AverageEncoderValue <= -20) {
    FrontLeftMotor.Set(0);
    FrontRightMotor.Set(0);
    
    

  }
}
if (autoChooser == "2") {
  if (autoStep == 1 && AverageEncoderValue <= 20) {
    FrontRightMotor.Set(0.20);
    FrontLeftMotor.Set(-0.20);

    autoStep++;

  }
  else if (autoStep == 2 && AverageEncoderValue >= 20) {
    FrontLeftMotor.Set(0);
    FrontRightMotor.Set(0);
  }
}
  // Score Preloaded Cone, then Leaving the community
  /*if (frc::SmartDashboard::GetNumber("Auto", 1) == 1)
  {
    if (autoStep == 1)
    {
      //ClawMotor.Set(.50);
      //ExtendMotorOne.SetSelectedSensorPosition(.25);
      //ExtendMotorTwo.SetSelectedSensorPosition(.25);
      //ArmMotorOne.Set(.25);
      //ClawMotor.Set(-0.50);

      autoStep++;
    }
    else if (autoStep == 2)
    {
      FrontRightMotor.Set(-0.25);
      FrontLeftMotor.Set(-0.25);
      autoStep++;
    }
    else if (autoStep == 3)
    {
      FrontRightMotor.Set(0.25);
      FrontLeftMotor.Set(0.25);
      autoStep++;
    }
  }
  // Auto for score preload, _______
  if (frc::SmartDashboard::GetNumber("Auto", 1) == 2)
  {
    if (autoStep == 1)
    {

      ClawMotor.Set(.50);
      //ExtendMotorOne.SetSelectedSensorPosition(.25);
      //ExtendMotorTwo.SetSelectedSensorPosition(.25);
      ArmMotorOne.Set(.25);
      ClawMotor.Set(-0.50);

      autoStep++;
    }
    else if (autoStep == 2)
    {
      FrontRightMotor.Set(-0.25);
      FrontLeftMotor.Set(-0.25);
      autoStep++;
    }
    else if (autoStep == 3)
    {
      FrontRightMotor.Set(0.25);
      FrontLeftMotor.Set(0.25);
      autoStep++;
    }
  }
  // Auto for only leaving the community
  if (frc::SmartDashboard::GetNumber("Auto", 1) == 3)
  {
    if (autoStep == 1)
    {
      FrontRightMotor.Set(-0.25);
      FrontLeftMotor.Set(-0.25);
      autoStep++;
    }
  }
  // Auto for scoring preload, then leaving the blue community, and then get another object
  if (frc::SmartDashboard::GetNumber("Auto", 1) == 4)
  {
    if (autoStep == 1)
    {
      ClawMotor.Set(.50);
      //ExtendMotorOne.SetSelectedSensorPosition(.25);
      //ExtendMotorTwo.SetSelectedSensorPosition(.25);
      ArmMotorOne.Set(.25);
      ClawMotor.Set(-0.50);
      autoStep++;
    }
    else if (autoStep == 2)
    {
      // 180 turn
      FrontRightMotor.Set(-0.5);
      FrontLeftMotor.Set(0.5);
      autoStep++;
    }
    else if (autoStep == 3)
    {
      FrontRightMotor.Set(0.25);
      FrontLeftMotor.Set(0.25);
      autoStep++;
    }
    else if (autoStep == 4)
    {
      IntakeMotorLeft.Set(0.25);
      IntakeMotorRight.Set(0.25);
      autoStep++;
    }
  }
  if (frc::SmartDashboard::GetNumber("Auto", 1) == 5)
  {
    if (autoStep == 1)
    {
      FrontRightMotor.Set(-0.25);
      FrontLeftMotor.Set(-0.25);
      autoStep++;
    }
    else if (autoStep == 2)
    {
      // 90 left turn at blue community
      FrontRightMotor.Set(0.25);
      FrontLeftMotor.Set(-0.25);
      autoStep++;
    }
    else if (autoStep == 3)
    {
      FrontRightMotor.Set(-0.25);
      FrontLeftMotor.Set(-0.25);
      autoStep++;
    }
    else if (autoStep == 4)
    {
      // 90 right turn at blue community
      FrontRightMotor.Set(-0.25);
      FrontLeftMotor.Set(0.25);
      autoStep++;
    }
    else if (autoStep == 5)
    {
      FrontRightMotor.Set(0.25);
      FrontLeftMotor.Set(0.25);
      autoStep++;
    }
  }
  // Auto for scoring preload, then leaving the Red community, and then get another object
  if (frc::SmartDashboard::GetNumber("Auto", 1) == 6)
  {
    if (autoStep == 1)
    {
      FrontRightMotor.Set(-0.25);
      FrontLeftMotor.Set(-0.25);
      autoStep++;
    }
    else if (autoStep == 2)
    {
      // 90 right turn at Red community
      FrontRightMotor.Set(-0.25);
      FrontLeftMotor.Set(0.25);
      autoStep++;
    }
    else if (autoStep == 3)
    {
      FrontRightMotor.Set(-0.25);
      FrontLeftMotor.Set(-0.25);
      autoStep++;
    }
    else if (autoStep == 4)
    {
      // 90 left turn at Red community
      FrontRightMotor.Set(0.25);
      FrontLeftMotor.Set(-0.25);
      autoStep++;
    }
    else if (autoStep == 5)
    {
      FrontRightMotor.Set(0.25);
      FrontLeftMotor.Set(0.25);
      autoStep++;
    }
    /*else if (autoStep == 6) {
      Likely is going to be PID control so that the charge station is engaged
      keep fixing gyro until gyro = 0
      autoStep++;
    */

  }

void Robot::TeleopInit()
{
  RightEncoder.SetPosition(0);
  LeftEncoder.SetPosition(0);

  if (m_autonomousCommand != nullptr)
  {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }
}

void Robot::TeleopPeriodic(){

AverageEncoderValue = (LeftEncoderValue + RightEncoderValue)/2;
LeftEncoderValue = -LeftEncoder.GetPosition();
RightEncoderValue = RightEncoder.GetPosition();


frc::SmartDashboard::PutString("DB/String 0", std::to_string(LeftEncoderValue));
frc::SmartDashboard::PutString("DB/String 1", std::to_string(RightEncoderValue));

std::cout << ("right encoder", RightEncoderValue);
std::cout << ("left encoder", LeftEncoderValue);

double ArmRotations = ArmEncoder.GetPosition();
  
frc::SmartDashboard::PutNumber("Arm Encoder Value", ArmRotations);

if (ArmRotations >= 0) {

}
/*
SuctionMotors.Set(false);
RollerMotor.Set(false);  
  if (Xbox.GetRawButtonPressed(0)) {
    SuctionMotors.Set(true);
    RollerMotor.Set(true); 
  }
  else if (Xbox.GetRawButton(1)) {
    SuctionMotors.Set(false);
    RollerMotor.Set(false);
  }
  if (Xbox.GetRawButton(0)) {
    ExtentionMotor.Set(.10);
  }
  else if (Xbox.GetRawButton(0)) {
    ExtentionMotor.Set(-.10);
  }

  */
  double WheelX = -Wheel.GetX();
  double JoyY = JoyStick1.GetY();
  FrontLeftMotor.Set((WheelX*0.4) + (0.3*JoyY));
  FrontRightMotor.Set((WheelX*0.4) - (0.3*JoyY));

  } 
void Robot::TestPeriodic()
{
  frc::SmartDashboard::PutString("DB/String 0", "This is a string");
  std::string dashData = frc::SmartDashboard::GetString("DB/String 0", "myDefaultData");
  frc::SmartDashboard::PutString("DB/String 1", dashData);
  std::string autoChooser = frc::SmartDashboard::GetString("DB/String 3", "myDefaultData");
  if(autoChooser == "1"){
    frc::SmartDashboard::PutString("DB/String 4", "One");
  }
  else{
    frc::SmartDashboard::PutString("DB/String 4", "Zero");
  } 

  /*SendableChooser<String> auto = new SendableChooser<String>();
   auto.addOption("JustLeaveCommunity", "JustLeaveCommunity");
        auto.addOption("Cit_Circuits", "Cit_Circuits");
        SmartDashboard.putData("Auto Mode", auto);
        String autonMode = auto.getSelected; */

  buttonValue = frc::SmartDashboard::GetBoolean("DB/Button 0", false);
  buttonValueTwo = frc::SmartDashboard::GetBoolean("DB/Button 1", false);
  buttonValueThree = frc::SmartDashboard::GetBoolean("DB/Button 2", false);
  buttonValueFour = frc::SmartDashboard::GetBoolean("DB/Button 3", false);

  if (buttonValue == true)
  {
    frc::SmartDashboard::PutBoolean("DB/Button 2", true);
  }
  else
  {
    frc::SmartDashboard::PutBoolean("DB/Button 2", false);
  }
  frc::SmartDashboard::PutNumber("Joystick Y value", JoyStick1.GetY());
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
