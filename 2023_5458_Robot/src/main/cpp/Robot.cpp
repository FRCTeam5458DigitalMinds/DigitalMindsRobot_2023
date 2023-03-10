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
#include <ctre/phoenixpro/Utils.hpp>
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
#include <ctre/Phoenix.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>

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

#include <ctre/phoenix/motorcontrol/SupplyCurrentLimitConfiguration.h>
#include <ctre/Phoenix.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h> 

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

//Gyro
//WPI_PigeonIMU gyro{};

//Drivetrain encoders


rev::SparkMaxRelativeEncoder LeftEncoder = FrontLeftMotor.GetEncoder();
rev::SparkMaxRelativeEncoder RightEncoder = FrontRightMotor.GetEncoder();


// Arm encoders
rev::CANSparkMax ArmUpOne{7, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax ArmUpTwo{8, rev::CANSparkMax::MotorType::kBrushless};
WPI_TalonFX ClawMotor{9};
WPI_TalonFX ExtensionMotorOne {10};
WPI_TalonFX ExtensionMotorTwo {11};

rev::SparkMaxRelativeEncoder ArmOneEncoder = ArmUpOne.GetEncoder();
rev::SparkMaxRelativeEncoder ArmTwoEncoder = ArmUpTwo.GetEncoder();

frc::Compressor pcmCompressor{0, frc::PneumaticsModuleType::CTREPCM};
// robot variables
double LeftEncoderValue;
double RightEncoderValue;
double ArmOneEncoderValue;
double ArmTwoEncoderValue;
double speed;
double AverageEncoderValue;
double AverageArmEncoderValue;

double maxextensionlimit;
double maxanglelimit = 4;


// set amp limit
bool enable = true;
double currentLimit = 30;
double triggerThresholdCurrent = 30;
double triggerThresholdTime = .1;

// Gyro
WPI_PigeonIMU gyro{12};

//compressors


WPI_TalonSRX SRX_1{15};
WPI_TalonSRX SRX_2{16};
WPI_TalonSRX SRX_3{17};

// Auto Variables
int autoStep = 1;
double extensionvalue;
// Timer
steady_clock::time_point clock_begin;

frc::Timer *shooterTimer;
bool timerStarted = false;

bool gyroResetted = false;

//dashboard variables
bool buttonValue;
bool buttonValueTwo;
bool buttonValueThree;
bool buttonValueFour;

// PID Aspects
double YAW;
double ROLL;
double PITCH;

//Intake Outake Variable
int bothTake = 1;

void Robot::RobotInit()
{
pcmCompressor.Disable();
ExtensionMotorOne.SetSelectedSensorPosition(0);
ExtensionMotorTwo.SetSelectedSensorPosition(0);
  //Reset encoders 
  RightEncoder.SetPosition(0);
  LeftEncoder.SetPosition(0);
  ArmOneEncoder.SetPosition(0);
  ArmTwoEncoder.SetPosition(0);
  // camera
  frc::CameraServer::StartAutomaticCapture();

  //Motor supply limits
  SupplyCurrentLimitConfiguration current_limit_config(enable, currentLimit, triggerThresholdCurrent, triggerThresholdTime);
  ExtensionMotorOne.ConfigSupplyCurrentLimit(current_limit_config);
  ExtensionMotorTwo.ConfigSupplyCurrentLimit(current_limit_config);
  ClawMotor.ConfigSupplyCurrentLimit(current_limit_config);

  // Follow front motors
  MiddleLeftMotor.Follow(FrontLeftMotor);
  BackLeftMotor.Follow(FrontLeftMotor);
  MiddleRightMotor.Follow(FrontRightMotor);
  BackRightMotor.Follow(FrontRightMotor);

  // Neo motor current limit
  FrontLeftMotor.SetSmartCurrentLimit(40);
  MiddleLeftMotor.SetSmartCurrentLimit(40);
  BackLeftMotor.SetSmartCurrentLimit(40);
  FrontRightMotor.SetSmartCurrentLimit(40);
  MiddleRightMotor.SetSmartCurrentLimit(40);
  BackRightMotor.SetSmartCurrentLimit(40);

  // Setting Idle Mode to brake (neo motors)
  FrontLeftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  MiddleLeftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  BackLeftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  FrontRightMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  MiddleRightMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  BackRightMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  ArmUpOne.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  ArmUpTwo.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

void Robot::RobotPeriodic()
{
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousInit()
{
pcmCompressor.Enabled();
ArmOneEncoder.SetPosition(0);
ArmTwoEncoder.SetPosition(0);
ExtensionMotorOne.SetSelectedSensorPosition(0);
ExtensionMotorTwo.SetSelectedSensorPosition(0);
  gyro.Reset();
  autoStep = 1;
  RightEncoder.SetPosition(0);
  LeftEncoder.SetPosition(0);
  ExtensionMotorOne.SetSelectedSensorPosition(0);
  ExtensionMotorTwo.SetSelectedSensorPosition(0);
  
  clock_begin = steady_clock::now();

  frc::CameraServer::StartAutomaticCapture();

  
}

void Robot::AutonomousPeriodic()
{
//pcmCompressor.Enabled();
ROLL = gyro.GetYaw();
PITCH = gyro.GetPitch();
ROLL = gyro.GetRoll();
frc::SmartDashboard::PutString("DB/String 8", ((std::to_string(ROLL))));
std::string autoChooser = frc::SmartDashboard::GetString("DB/String 2", "myDefaultData");
if (PITCH <= 10 || PITCH >= -10) {
  if(autoChooser == "1"){
    frc::SmartDashboard::PutString("DB/String 3", "AutoOne");
  }
  else if(autoChooser == "2"){
    frc::SmartDashboard::PutString("DB/String 3", "AutoTwo");
  }
  else if(autoChooser == "3"){
    frc::SmartDashboard::PutString("DB/String 3", "AutoThree");
  }
}
  AverageEncoderValue = (LeftEncoderValue + RightEncoderValue)/2;

  LeftEncoderValue = -LeftEncoder.GetPosition();
  RightEncoderValue = RightEncoder.GetPosition();

  frc::SmartDashboard::PutString("DB/String 7", ("Left: " + std::to_string(AverageEncoderValue)));
  frc::SmartDashboard::PutString("DB/String 0", ("Left: " + std::to_string(LeftEncoderValue)));
  frc::SmartDashboard::PutString("DB/String 1", ("Right: " + std::to_string(RightEncoderValue)));

  //Only going on charge station
if (autoChooser == "1") {
  
  //Goes backwards towards the charge station inversed motors
  if (autoStep == 1 && AverageEncoderValue >= -26.5) {
    speed = 0.4;
    FrontRightMotor.Set(-speed);
    FrontLeftMotor.Set(speed);

    autoStep++;

  }
  //makes robot stop on top of charge station
  else if (autoStep == 2 && AverageEncoderValue <= -26.5) {
    ROLL = gyro.GetYaw();
    if (ROLL <= 10) {
      if (ROLL > 3) {
        FrontLeftMotor.Set(0.2);
        FrontRightMotor.Set(-0.2);
      }
      if (ROLL < 3) {
        FrontLeftMotor.Set(0.05);
        FrontRightMotor.Set(-0.05);
    
    }
    if (ROLL >= 10) {
      if (ROLL < 3) {
        FrontLeftMotor.Set(-0.2);
        FrontRightMotor.Set(0.2);
      }
      if (ROLL > 3) {
        FrontLeftMotor.Set(-0.05);
        FrontRightMotor.Set(0.05);        
      }
  }
}
  
//forwards5ft Out of the community
if (autoChooser == "2") {
  if (autoStep == 1 && AverageEncoderValue <= 27) {
    speed = 0.4;
    FrontRightMotor.Set(speed);
    FrontLeftMotor.Set(-speed);

    autoStep++;

  }
  else if (autoStep == 2 && AverageEncoderValue >= 27) {
    FrontLeftMotor.Set(0);
    FrontRightMotor.Set(0);
  }
}
//Auto to leave community and get onto the charge station
if (autoChooser == "3") {
  //out of community
  if (autoStep == 1 && AverageEncoderValue <= 49) {
    speed = 0.2;
    FrontLeftMotor.Set(-speed);
    FrontRightMotor.Set(speed);
    
  }
  else if(autoStep == 1 && AverageEncoderValue >= 49) {
    
    autoStep++;
  }
  //back onto charge station
  else if(autoStep == 2 && AverageEncoderValue >= 24) {
    speed = -0.2;
    FrontLeftMotor.Set(speed);
    FrontRightMotor.Set(-speed);
  }
  else if(autoStep == 2 && AverageEncoderValue <= 24) {
    autoStep++;
  }
  else if(autoStep == 3) {

    //so this is to get onto the charging station and autocorrect itself
    //hi
    ROLL = gyro.GetYaw();
  if (ROLL <= 10) {
      if (ROLL > 3) {
        FrontLeftMotor.Set(0.2);
        FrontRightMotor.Set(-0.2);
      }
      if (ROLL < 3) {
        FrontLeftMotor.Set(0.05);
        FrontRightMotor.Set(-0.05);
    
    }
    if (ROLL >= 10) {
      if (ROLL < 3) {
        FrontLeftMotor.Set(-0.2);
        FrontRightMotor.Set(0.2);
      }
      if (ROLL > 3) {
        FrontLeftMotor.Set(-0.05);
        FrontRightMotor.Set(0.05);        
      }
    }
  }
  }
if (autoChooser == "4") {
  //Arm and Extension (Scoring during auto)
  if (autoStep == 1) {
    ArmUpOne.Set(0.2);
    ArmUpTwo.Set(-0.2);
    ExtensionMotorOne.Set(0.5);
    ExtensionMotorTwo.Set(0.5); 
  }
  //this will go past the charge station
  else if (autoStep == 2 && AverageEncoderValue >= -49) {
    speed = 0.4;
    FrontRightMotor.Set(speed); 
    FrontLeftMotor.Set(-speed); 
  } 
  else if (autoStep == 2 && AverageEncoderValue <= -49){
    autoStep++;
  }
  //this will go onto the charge station
  else if (autoStep == 3 && AverageEncoderValue <= -24) { 
    speed = 0.2;
    FrontRightMotor.Set(speed); 
    FrontLeftMotor.Set(-speed); 
  } 
  else if (autoStep == 3 && AverageEncoderValue >= -24) {
    autoStep++; 
  }
  else if (autoStep == 4) {
if (ROLL <= 10) {
      if (ROLL > 3) {
        FrontLeftMotor.Set(0.2);
        FrontRightMotor.Set(-0.2);
      }
      if (ROLL < 3) {
        FrontLeftMotor.Set(0.05);
        FrontRightMotor.Set(-0.05);
    
    }
    if (ROLL >= 10) {
      if (ROLL < 3) {
        FrontLeftMotor.Set(-0.2);
        FrontRightMotor.Set(0.2);
      }
      if (ROLL > 3) {
        FrontLeftMotor.Set(-0.05);
        FrontRightMotor.Set(0.05);        
      }
      }
    }
  }
}
  //if (frc::SmartDashboard::GetNumber("Auto", 1) == 5)
  if (autoChooser == "5") {
    //Arm and Extension (Scoring during auto)
  if (autoStep == 1) {
    ArmUpOne.Set(0.2);
    ArmUpTwo.Set(-0.2);
    ExtensionMotorOne.Set(0.5);
    ExtensionMotorTwo.Set(0.5); 
  }
  //this will go past the charge station
  else if (autoStep == 2 && AverageEncoderValue >= -49) {
    speed = 0.4;
    FrontRightMotor.Set(speed); 
    FrontLeftMotor.Set(-speed); 
  } 
  else if (autoStep == 2 && AverageEncoderValue <= -49){
    autoStep++;
  }
  //this will go onto the charge station
  else if (autoStep == 3 && AverageEncoderValue <= -24) { 
    speed = 0.2;
    FrontRightMotor.Set(speed); 
    FrontLeftMotor.Set(-speed); 
  } 
  else if (autoStep == 3 && AverageEncoderValue >= -24) {
    autoStep++; 
  }
  else if (autoStep == 4) {
    ROLL = gyro.GetYaw();
    if (ROLL <= 10) {
      if (ROLL > 3) {
        FrontLeftMotor.Set(0.2);
        FrontRightMotor.Set(-0.2);
      }
      if (ROLL < 3) {
        FrontLeftMotor.Set(0.05);
        FrontRightMotor.Set(-0.05);
    
    }
    if (ROLL >= 10) {
      if (ROLL < 3) {
        FrontLeftMotor.Set(-0.2);
        FrontRightMotor.Set(0.2);
      }
      if (ROLL > 3) {
        FrontLeftMotor.Set(-0.05);
        FrontRightMotor.Set(0.05);        
      }
    }
  }
  }
}
if (autoChooser == "6") {
   //Arm and Extension (Scoring during auto)
  if (autoStep == 1) {
    ArmUpOne.Set(0.2);
    ArmUpTwo.Set(-0.2);
    ExtensionMotorOne.Set(0.5);
    ExtensionMotorTwo.Set(0.5); 
  }
  //this will go out of the community
  else if (autoStep == 2 && AverageEncoderValue >= -49) {
    speed = 0.4;
    FrontRightMotor.Set(speed); 
    FrontLeftMotor.Set(-speed); 
  } 
  else if (autoStep == 2 && AverageEncoderValue <= -49){
    autoStep++;
  }
  else if (autoStep == 4) {
    FrontRightMotor.Set(0);
    FrontLeftMotor.Set(0);
  }
  }
  }
  }
else {
  double PIDAngle = -gyro.GetPitch();
  FrontRightMotor.Set((PIDAngle * (speed*0.67)) - (speed));
  FrontLeftMotor.Set((PIDAngle * (speed*0.67)) + (speed));
}
  }



}

void Robot::TeleopInit()
{
 
  ArmOneEncoder.SetPosition(0);
  ArmTwoEncoder.SetPosition(0);
  ExtensionMotorOne.SetSelectedSensorPosition(0);
  ExtensionMotorTwo.SetSelectedSensorPosition(0);
  //Resetting sensor positions using buttons on the dashboard
    buttonValueThree = frc::SmartDashboard::GetBoolean("DB/Button 2", false);
  if (buttonValueThree == true)
  {
    ExtensionMotorOne.SetSelectedSensorPosition(0);
    ExtensionMotorTwo.SetSelectedSensorPosition(0);
  }
    buttonValueFour = frc::SmartDashboard::GetBoolean("DB/Button 3", false);

  if (buttonValueFour == true)
  {
    ArmOneEncoder.SetPosition(0);
    ArmTwoEncoder.SetPosition(0);
  }
  
  RightEncoder.SetPosition(0);
  LeftEncoder.SetPosition(0);
  gyro.Reset();

  if (m_autonomousCommand != nullptr)
  {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;

  }
  
}

void Robot::TeleopPeriodic(){
pcmCompressor.EnableDigital();
extensionvalue = ExtensionMotorOne.GetSelectedSensorPosition();

ROLL = gyro.GetRoll();
frc::SmartDashboard::PutString("DB/String 8", ((std::to_string(ROLL))));
frc::SmartDashboard::PutString("DB/String 4", ((std::to_string(extensionvalue))));

AverageEncoderValue = (LeftEncoderValue + RightEncoderValue)/2;
LeftEncoderValue = -LeftEncoder.GetPosition();
RightEncoderValue = RightEncoder.GetPosition();

AverageArmEncoderValue = (ArmTwoEncoderValue + ArmOneEncoderValue)/2;
ArmOneEncoderValue = -ArmOneEncoder.GetPosition();
ArmTwoEncoderValue = ArmTwoEncoder.GetPosition();

ExtensionMotorOne.Set(0);
ExtensionMotorTwo.Set(0);

if (AverageArmEncoderValue <= -10) {
  maxextensionlimit = 100000;
}
if (AverageArmEncoderValue <= -20) {
  maxextensionlimit = 71000;
}
if (AverageArmEncoderValue <= -30) {
  maxextensionlimit = 35859.636;
}
else {
  maxextensionlimit = 143438.544;
}


if (extensionvalue <= maxextensionlimit && extensionvalue >= 0) {
  if (Xbox.GetRawButton(1)) {
  ExtensionMotorOne.Set(0.3);
  ExtensionMotorTwo.Set(0.3);
}
  else if (Xbox.GetRawButton(2)) {
    ExtensionMotorOne.Set(-0.3);
    ExtensionMotorTwo.Set(-0.3);
  }
  else {
    ExtensionMotorOne.Set(0);
    ExtensionMotorTwo.Set(0);
  }
} 
if (extensionvalue >= maxextensionlimit){
  if (Xbox.GetRawButton(2)){
    ExtensionMotorOne.Set(-0.3);
    ExtensionMotorTwo.Set(-0.3);
  }
  else {
    ExtensionMotorOne.Set(-0.1);
    ExtensionMotorTwo.Set(-0.1);
  }
}
if (extensionvalue <= 0){
  if (Xbox.GetRawButton(1)){
    ExtensionMotorOne.Set(0.3);
    ExtensionMotorOne.Set(0.3);
  }
  else {
    ExtensionMotorOne.Set(0);
    ExtensionMotorTwo.Set(0);
  }
}
if ((AverageArmEncoderValue >= -47) && (AverageArmEncoderValue <= 0)) {
if (Xbox.GetRawButton(6)) {
  ArmUpOne.Set(-0.1);
  ArmUpTwo.Set(0.1);
}
else if (Xbox.GetRawButton(5)) {
  ArmUpOne.Set(0.1);
  ArmUpTwo.Set(-0.1);
}
else {
  ArmUpOne.Set(0);
  ArmUpTwo.Set(0);
}
}

if (AverageArmEncoderValue <= -47) {
  if (Xbox.GetRawButton(5)) {
    ArmUpOne.Set(-0.1);
    ArmUpTwo.Set(0.1);
  }
  else {
    ArmUpOne.Set(0);
    ArmUpTwo.Set(0);
  }
}
if (AverageArmEncoderValue >= 0) {
  if (Xbox.GetRawButton(6)) {
    ArmUpOne.Set(0.1);
    ArmUpTwo.Set(-0.1);
  }
  else {
    ArmUpOne.Set(0);
    ArmUpTwo.Set(0);
  }
}
frc::SmartDashboard::PutString("DB/String 7", ("Average" + std::to_string(AverageEncoderValue)));


frc::SmartDashboard::PutString("DB/String 5", std::to_string(ArmOneEncoderValue));
frc::SmartDashboard::PutString("DB/String 6", std::to_string(ArmTwoEncoderValue));

frc::SmartDashboard::PutString("DB/String 0", std::to_string(LeftEncoderValue));
frc::SmartDashboard::PutString("DB/String 1", std::to_string(RightEncoderValue));
//Turing the suction motors off
SRX_1.Set(false);
SRX_2.Set(false);
SRX_3.Set(false);
ClawMotor.Set(0); 

if (Xbox.GetRawButtonPressed(3)) {
  bothTake = 2;
}
if (Xbox.GetRawButtonPressed(4)) {
  bothTake = 3;
}
// turning it on based on the button pressed
  if (bothTake == 2) {
    SRX_1.Set(0.4);
    SRX_2.Set(0.4);
    SRX_3.Set(0.4);
    ClawMotor.Set(0.3); 
  }
  //toggling the button off based on the button pressed
  else if (bothTake == 3) {
    SRX_1.Set(false);
    SRX_2.Set(false);
    SRX_3.Set(false);
    ClawMotor.Set(-0.3);
    sleep(3);
    bothTake = 1;
  }
  else if (bothTake == 1) {
    SRX_1.Set(false);
    SRX_2.Set(false);
    SRX_3.Set(false);
    ClawMotor.Set(0);
  }
  //setting base values for teleop
  double WheelX = -Wheel.GetX();
  double JoyY = JoyStick1.GetY();
  FrontLeftMotor.Set((WheelX*0.4) + (0.6*JoyY));
  FrontRightMotor.Set((WheelX*0.4) - (0.6*JoyY));

  

  } 
void Robot::TestPeriodic()
{

if (Xbox.GetRawButton(4)) {
  ExtensionMotorOne.Set(0.5);
  ExtensionMotorTwo.Set(0.5);
}
  else if (Xbox.GetRawButton(1)) {
    ExtensionMotorOne.Set(-0.5);
    ExtensionMotorTwo.Set(-0.5);
  }
  else {
    ExtensionMotorOne.Set(0);
    ExtensionMotorTwo.Set(0);
  }
AverageArmEncoderValue = (ArmTwoEncoderValue + ArmOneEncoderValue)/2;
ArmOneEncoderValue = -ArmOneEncoder.GetPosition();
ArmTwoEncoderValue = ArmTwoEncoder.GetPosition();
if (Xbox.GetRawButton(5)) {
  ArmUpOne.Set(-0.02);
  ArmUpTwo.Set(0.02);
}
else if (Xbox.GetRawButton(6)) {
  ArmUpOne.Set(0.02);
  ArmUpTwo.Set(-0.02);
}
else {
  ArmUpOne.Set(0);
  ArmUpTwo.Set(0);
}
AverageArmEncoderValue = (ArmTwoEncoderValue + ArmOneEncoderValue)/2;
ArmOneEncoderValue = -ArmOneEncoder.GetPosition();
ArmTwoEncoderValue = ArmTwoEncoder.GetPosition();

buttonValue = frc::SmartDashboard::GetBoolean("DB/Button 0", false);
buttonValueTwo = frc::SmartDashboard::GetBoolean("DB/Button 1", false);
buttonValueThree = frc::SmartDashboard::GetBoolean("DB/Button 2", false);
buttonValueFour = frc::SmartDashboard::GetBoolean("DB/Button 3", false);

if (buttonValueFour == true)
  {
    ArmOneEncoder.SetPosition(0);
    ArmTwoEncoder.SetPosition(0);
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

