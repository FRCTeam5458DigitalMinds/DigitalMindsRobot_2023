// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Libraries
#include "Robot.h"
#include "RobotContainer.h"
#include "cameraserver/CameraServer.h"

#include <frc/filter/SlewRateLimiter.h>
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

// Intake Motor (New Intake)
// rev::CANSparkMax Intake{25, rev::CANSparkMax::MotorType::kBrushless};

// Drivetrain encoders
rev::SparkMaxRelativeEncoder LeftEncoder = FrontLeftMotor.GetEncoder();
rev::SparkMaxRelativeEncoder RightEncoder = FrontRightMotor.GetEncoder();

// Arm encoders
rev::CANSparkMax ArmUpOne{7, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax ArmUpTwo{8, rev::CANSparkMax::MotorType::kBrushless};
WPI_TalonFX ClawMotor{9};
WPI_TalonFX ExtensionMotorOne{10};
WPI_TalonFX ExtensionMotorTwo{11};
WPI_TalonFX Intake{25};

rev::SparkMaxRelativeEncoder ArmOneEncoder = ArmUpOne.GetEncoder();
rev::SparkMaxRelativeEncoder ArmTwoEncoder = ArmUpTwo.GetEncoder();

frc::Compressor pcmCompressor{0, frc::PneumaticsModuleType::CTREPCM};
frc::Solenoid Piston{frc::PneumaticsModuleType::CTREPCM, 0};
frc::Solenoid IntakePiston{frc::PneumaticsModuleType::CTREPCM, 1};
frc::Solenoid Lights{frc::PneumaticsModuleType::CTREPCM, 3};

frc::Solenoid Vent1{frc::PneumaticsModuleType::CTREPCM, 5};
frc::Solenoid Vent2{frc::PneumaticsModuleType::CTREPCM, 6};
frc::Solenoid Vent3{frc::PneumaticsModuleType::CTREPCM, 7};

// robot variables
double LeftEncoderValue;
double RightEncoderValue;
double ArmOneEncoderValue;
double ArmTwoEncoderValue;
double speed;
double AverageEncoderValue;
double AverageArmEncoderValue;

bool autoStart = false;
double maxextensionlimit;
double mainlimit;
double maxanglelimit = 4;
bool subpreset = false;
bool pistonenable = false;

// set amp limit
bool enable = true;
bool brakemode = false;
double currentLimit = 60;
double triggerThresholdCurrent = 60;
double triggerThresholdTime = .1;

double currentarm;
double currentextend;
double limitfactor;
// Gyro
WPI_PigeonIMU gyro{12};

// compressors
WPI_TalonSRX SRX_1{15};
WPI_TalonSRX SRX_2{16};
WPI_TalonSRX SRX_3{17};

// Auto Variables
int autoStep = 1;
double extensionvalue;
// Timer
steady_clock::time_point clock_begin;

frc::Timer *shooterTimer;
frc::Timer *driveTimer;
bool gyroResetted = false;

// dashboard variables
bool buttonValue;
bool buttonValueTwo;
bool buttonValueThree;
bool buttonValueFour;
double k_rampTimeSeconds = 0.25;

units::scalar_t vFactor;

bool timerStarted = false;

double highscorearm = -16.93;
double highscoreextend = 72000;
double mediumscorearm;
double mediumscoreextend;

double goalextend;

// PID Aspects
double YAW;
double ROLL;
double PITCH;
double SPEED;
double driveLimit;

bool coneintake;

// Intake Outake Variable
int bothTake = 1;
frc::Timer *highTimer;
frc::Timer *ventTimer;
frc::Timer *pistonTimer;
frc::Timer *clawTimer;

void Robot::RobotInit()
{
  IntakePiston.Set(false);
  gyro.Reset();
  Piston.Set(0);

  gyro.Calibrate();
  pcmCompressor.Disable();
  ExtensionMotorOne.SetSelectedSensorPosition(0);
  ExtensionMotorTwo.SetSelectedSensorPosition(0);

  // Reset encoders
  RightEncoder.SetPosition(0);
  LeftEncoder.SetPosition(0);
  ArmOneEncoder.SetPosition(0);
  ArmTwoEncoder.SetPosition(0);

  // camera
  frc::CameraServer::StartAutomaticCapture();

  // Motor supply limits
  SupplyCurrentLimitConfiguration current_limit_config(enable, currentLimit, triggerThresholdCurrent, triggerThresholdTime);
  SupplyCurrentLimitConfiguration current_claw_config(enable, 30, 30, triggerThresholdTime);
  ExtensionMotorOne.ConfigSupplyCurrentLimit(current_limit_config);
  ExtensionMotorTwo.ConfigSupplyCurrentLimit(current_limit_config);
  ClawMotor.ConfigSupplyCurrentLimit(current_claw_config);

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

  pistonTimer = new frc::Timer();
  clawTimer = new frc::Timer();
}

void Robot::RobotPeriodic()
{
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit()
{
}

void Robot::DisabledPeriodic()
{
}

void Robot::AutonomousInit()
{

  clawTimer->Reset();
  // Enabling the PCM compressor/Pnuematically controlled
  pcmCompressor.EnableDigital();
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

  gyro.SetYaw(0);
  clock_begin = steady_clock::now();

  frc::CameraServer::StartAutomaticCapture();

  timerStarted = false;
}

void Robot::AutonomousPeriodic()
{
  // Enabling the PCM compressor
  pcmCompressor.Enabled();
  Piston.Set(0);
  YAW = gyro.GetYaw();
  PITCH = gyro.GetPitch();
  ROLL = gyro.GetRoll() - 2;
  frc::SmartDashboard::PutString("DB/String 8", ((std::to_string(ROLL))));
  // frc::SmartDashboard::PutString("DB/String 8", ((std::to_string(ROLL))));
  frc::SmartDashboard::PutString("DB/String 9", ((std::to_string(YAW))));
  frc::SmartDashboard::PutString("DB/String 3", ((std::to_string(maxextensionlimit))));
  std::string autoChooser = frc::SmartDashboard::GetString("DB/String 2", "myDefaultData");

  if (autoChooser == "1")
  {
    frc::SmartDashboard::PutString("DB/String 3", "AutoOne");
  }
  else if (autoChooser == "2")
  {
    frc::SmartDashboard::PutString("DB/String 3", "AutoTwo");
  }
  else if (autoChooser == "3")
  {
    frc::SmartDashboard::PutString("DB/String 3", "AutoThree");
  }

  AverageEncoderValue = (LeftEncoderValue + RightEncoderValue) / 2;

  LeftEncoderValue = -LeftEncoder.GetPosition();
  RightEncoderValue = RightEncoder.GetPosition();

  frc::SmartDashboard::PutString("DB/String 7", ("Left: " + std::to_string(AverageEncoderValue)));
  frc::SmartDashboard::PutString("DB/String 0", ("Left: " + std::to_string(LeftEncoderValue)));
  frc::SmartDashboard::PutString("DB/String 1", ("Right: " + std::to_string(RightEncoderValue)));

  AverageArmEncoderValue = (ArmTwoEncoderValue + ArmOneEncoderValue) / 2;
  ArmOneEncoderValue = -ArmOneEncoder.GetPosition();
  ArmTwoEncoderValue = ArmTwoEncoder.GetPosition();

  extensionvalue = ExtensionMotorOne.GetSelectedSensorPosition();
  // Auto 1: Only going on charge station: Front Side and No Scoring
  if (autoChooser == "1")
  {
    // Goes forwards towards the charge station inversed motors
    if (autoStep == 1 && AverageEncoderValue <= 26.5)
    {
      speed = 0.4;
      FrontRightMotor.Set(speed);
      FrontLeftMotor.Set(-speed);

      autoStep++;
    }
    // makes robot stop on top of charge station
    else if (autoStep == 2 && AverageEncoderValue >= 26.5)
    {
      ROLL = gyro.GetRoll() - 2;

      if (ROLL >= -1 && ROLL <= 1)
      {
        FrontLeftMotor.Set(0);
        FrontRightMotor.Set(0);
      }
      if (ROLL >= 1)
      {
        FrontLeftMotor.Set(0.3);
        FrontRightMotor.Set(-0.3);
      }
      else if (ROLL <= -1)
      {
        FrontLeftMotor.Set(-0.3);
        FrontRightMotor.Set(0.3);
      }
    }
  }
  // Auto 2: forwards5ft Out of the community
  if (autoChooser == "2")
  {
    if (autoStep == 1)
    {
      IntakePiston.Set(false);
      Intake.Set(-0.3);
      sleep(2);
      Intake.Set(0);
      autoStep++;
    }

    if (autoStep == 2 && AverageEncoderValue >= -37)
    {
      speed = 0.4;
      FrontRightMotor.Set(-speed);
      FrontLeftMotor.Set(speed);
      autoStep++;
    }
    else if (autoStep == 3 && AverageEncoderValue <= -37)
    {
      FrontLeftMotor.Set(0);
      FrontRightMotor.Set(0);
    }
  }

  // Auto 3: Score and leave the community
  if (autoChooser == "3")
  {

    // Arm and Extension (Scoring during auto)
    extensionvalue = ExtensionMotorOne.GetSelectedSensorPosition();

    if (autoStep == 1)
    {
      ClawMotor.Set(0.3);
      SRX_1.Set(true);
      SRX_2.Set(true);
      SRX_3.Set(true);
      Vent1.Set(false);
      Vent2.Set(false);
      Vent3.Set(false);
      if (AverageArmEncoderValue >= highscorearm)
      {
        ArmUpOne.Set(0.2);
        ArmUpTwo.Set(-0.2);
      }
      else
      {
        ArmUpOne.Set(0);
        ArmUpTwo.Set(0);
      }
      if (extensionvalue <= highscoreextend)
      {
        ExtensionMotorOne.Set(0.3);
        ExtensionMotorTwo.Set(0.3);
      }
      else
      {
        ExtensionMotorOne.Set(0);
        ExtensionMotorTwo.Set(0);
      }
      if (extensionvalue >= highscoreextend && AverageArmEncoderValue <= highscorearm)
      {
        autoStep++;
      }
    }
    // this will drop the cube
    else if (autoStep == 2)
    {
      if (!timerStarted)
      {
        timerStarted = true;
        clawTimer->Reset();
        clawTimer->Start();
      }
      if (clawTimer->Get() < (units::time::second_t)2)
      {
        SRX_1.Set(false);
        SRX_2.Set(false);
        SRX_3.Set(false);
        Vent1.Set(true);
        Vent2.Set(true);
        Vent3.Set(true);
        ClawMotor.Set(-0.3);
      }

      if (clawTimer->Get() > (units::time::second_t)2)
      {
        if (AverageArmEncoderValue <= -2)
        {
          ArmUpOne.Set(-0.2);
          ArmUpTwo.Set(0.2);
        }
        else
        {
          ArmUpOne.Set(0);
          ArmUpTwo.Set(0);
        }
        if (extensionvalue >= 3000)
        {
          ExtensionMotorOne.Set(-0.3);
          ExtensionMotorTwo.Set(-0.3);
        }
        else
        {
          ExtensionMotorOne.Set(0);
          ExtensionMotorTwo.Set(0);
        }
      }
    }
    // this will go out of the community
    else if (autoStep == 3)
    {
      if (YAW <= 3 && YAW >= -3)
      {
        if (autoStep == 2 && AverageEncoderValue >= -30)
        {
          speed = -0.3;
          FrontRightMotor.Set(speed);
          FrontLeftMotor.Set(-speed);
        }
      }
      else
      {
        if (AverageEncoderValue >= -30)
        {
          if (YAW >= 3)
          {
            FrontRightMotor.Set(speed);
            FrontLeftMotor.Set(-speed * 0.7);
          }
          if (YAW <= -3)
          {
            FrontRightMotor.Set(speed * 0.7);
            FrontLeftMotor.Set(-speed);
          }
        }
      }
    }
  }
  if (autoChooser == "hamburgerwithcheese")
  {
    FrontLeftMotor.Set(speed);
    FrontRightMotor.Set(speed);
  }
  // Auto 4: Scoring High
  if (autoChooser == "4")
  {
    // Arm and Extension (Scoring during auto)
    if (autoStep == 1)
    {
      if (AverageArmEncoderValue >= highscorearm)
      {
        ArmUpOne.Set(0.2);
        ArmUpTwo.Set(-0.2);
      }
      else
      {
        ArmUpOne.Set(0);
        ArmUpTwo.Set(0);
      }
      if (extensionvalue <= highscoreextend)
      {
        ExtensionMotorOne.Set(0.3);
        ExtensionMotorTwo.Set(0.3);
      }
      else
      {
        ExtensionMotorOne.Set(0);
        ExtensionMotorTwo.Set(0);
      }
      if (extensionvalue >= highscoreextend && AverageArmEncoderValue <= highscorearm)
      {
        autoStep++;
      }
    }
    else if (autoStep == 2)
    {
      if (!timerStarted)
      {
        timerStarted = true;
        clawTimer->Reset();
        clawTimer->Start();
      }
      if (clawTimer->Get() < (units::time::second_t)2)
      {
        SRX_1.Set(false);
        SRX_2.Set(false);
        SRX_3.Set(false);
        ClawMotor.Set(-0.3);
      }

      if (clawTimer->Get() > (units::time::second_t)2)
      {
        if (AverageArmEncoderValue <= 7)
        {
          ArmUpOne.Set(-0.2);
          ArmUpTwo.Set(0.2);
        }
        else
        {
          ArmUpOne.Set(0);
          ArmUpTwo.Set(0);
        }
        if (extensionvalue >= 3000)
        {
          ExtensionMotorOne.Set(-0.3);
          ExtensionMotorTwo.Set(-0.3);
        }
        else
        {
          ExtensionMotorOne.Set(0);
          ExtensionMotorTwo.Set(0);
        }
      }
    }
  }
  if (autoChooser == "5")
  {
    if (autoStep == 1)
    {
      IntakePiston.Set(false);
      Intake.Set(-0.4);
      sleep(1.5);
      Intake.Set(0);
      autoStep++;
      /*ClawMotor.Set(0.5);
      SRX_1.Set(true);
      SRX_2.Set(true);
      SRX_3.Set(true);
      Vent1.Set(false);
      Vent1.Set(false);
      Vent1.Set(false);
      sleep(0.5);
      ClawMotor.Set(-0.9);
      SRX_1.Set(false);
      SRX_2.Set(false);
      SRX_3.Set(false);
      Vent1.Set(true);
      Vent2.Set(true);
      Vent3.Set(true);
      sleep(2);
      ClawMotor.Set(0);*/
    }
    // Leave Community
    if (autoStep == 2)
    {
      if (YAW <= 3 && YAW >= -3)
      {
        if (autoStep == 2 && AverageEncoderValue >= -45)
        {
          speed = -0.3;
          FrontRightMotor.Set(speed * 0.9);
          FrontLeftMotor.Set(-speed);
        }
        else
        {
          FrontRightMotor.Set(0);
          FrontLeftMotor.Set(0);
        }
      }
      else
      {
        if (AverageEncoderValue >= -45)
        {
          if (YAW >= 3)
          {
            FrontRightMotor.Set(speed);
            FrontLeftMotor.Set(-speed * 0.7);
          }
          if (YAW <= -3)
          {
            FrontRightMotor.Set(speed * 0.7);
            FrontLeftMotor.Set(-speed);
          }
        }
        else
        {
          FrontRightMotor.Set(0);
          FrontLeftMotor.Set(0);
        }
      }
    }
  }
  // Auto 6: Scores mid/low and goes onto charge station
  if (autoChooser == "6")
  {
    if (autoStep == 1)
    {
      IntakePiston.Set(false);
      Intake.Set(-0.4);
      sleep(1.5);
      Intake.Set(0);
      /*ClawMotor.Set(0.8);
      SRX_1.Set(true);
      SRX_2.Set(true);
      SRX_3.Set(true);
      Vent1.Set(false);
      Vent1.Set(false);
      Vent1.Set(false);
      sleep(0.5);
      ClawMotor.Set(-1);
      SRX_1.Set(false);
      SRX_2.Set(false);
      SRX_3.Set(false);
      Vent1.Set(true);
      Vent2.Set(true);
      Vent3.Set(true);
      sleep(2);
      ClawMotor.Set(0);
      gyro.SetYaw(0);*/
      autoStep++;
    }
    if (autoStep == 2)
    {
      if (YAW <= 173)
      {
        speed = 0.15;
        FrontLeftMotor.Set(speed);
        FrontRightMotor.Set(speed);
      }
      else
      {
        FrontLeftMotor.Set(0);
        FrontRightMotor.Set(0);
        RightEncoder.SetPosition(0);
        LeftEncoder.SetPosition(0);
        sleep(0.5);
        autoStep++;
      }
    }
    if (autoStep == 3 && AverageEncoderValue < 31)
    {
      speed = 0.5;
      FrontRightMotor.Set(speed);
      FrontLeftMotor.Set(-speed);
    }
    if (autoStep == 3 && AverageEncoderValue >= 31)
   {
      ROLL = gyro.GetRoll() - 2;
      if (ROLL >= -3 && ROLL <= 3)
      {
        FrontLeftMotor.Set(0);
        FrontRightMotor.Set(0);
      }
      if (ROLL >= 3)
      {
        if (YAW <= 183 && YAW >= 177)
        {
          FrontLeftMotor.Set(0.1);
          FrontRightMotor.Set(-0.09368259);
        }
        else if (YAW >= 183)
        {
          FrontLeftMotor.Set(0.1);
          FrontRightMotor.Set(-0.07);
        }
        else if (YAW <= 177)
        {
          FrontLeftMotor.Set(0.07);
          FrontRightMotor.Set(-0.1);
        }
      }
      else if (ROLL <= -3)
      {
        if (YAW <= 183 && YAW >= 177)
        {
          FrontLeftMotor.Set(-0.1);
          FrontRightMotor.Set(0.09368259);
        }
        else if (YAW >= 183)
        {
          FrontLeftMotor.Set(-0.07);
          FrontRightMotor.Set(0.1);
        }
        else if (YAW <= 177)
        {
          FrontLeftMotor.Set(-0.1);
          FrontRightMotor.Set(0.07);
        }
      }
    }
  }
  if (autoChooser == "7")
  {
    IntakePiston.Set(false);
    Intake.Set(-0.4);
    sleep(1.5);
    Intake.Set(0);
    autoStep++;
  }

  if (autoChooser == "9")
  {
    // dispersing cube
    if (autoStep == 1)
    {
      Intake.Set(-0.48);
      sleep(1.5);
      Intake.Set(0);
      autoStep = 2;
    }
    // 180 degree turn
    if (autoStep == 2)
    {
      if (YAW <= 173)
      {
        speed = 0.15;
        FrontLeftMotor.Set(speed);
        FrontRightMotor.Set(speed);
      }
      else
      {
        FrontLeftMotor.Set(0);
        FrontRightMotor.Set(0);
        LeftEncoder.SetPosition(0);
        RightEncoder.SetPosition(0);
        autoStep = 3;
      }
    }
    // over charging station
    if (autoStep == 3 && AverageEncoderValue < 40)
    {
      speed = 0.3;
      FrontRightMotor.Set(speed * 0.9);
      FrontLeftMotor.Set(-speed);
    }
    else
    {
      autoStep++;
    }
    // turning again
    if (autoStep == 4)
    {
      if (YAW >= 7)
      {
        speed = -0.15;
        FrontLeftMotor.Set(speed);
        FrontRightMotor.Set(speed);
      }
      else
      {
        gyro.SetYaw(0);
        LeftEncoder.SetPosition(0);
        RightEncoder.SetPosition(0);
        autoStep++;
      }
    }
    // onto charging station
    if (autoStep == 5 && AverageEncoderValue < 15)
    {
      speed = 0.3;
      FrontRightMotor.Set(speed);
      FrontLeftMotor.Set(-speed * 0.9);
    }
    // balancing charging station
    if (autoStep == 5 && AverageEncoderValue >= 15)
    {
      ROLL = gyro.GetRoll() - 2;
      if (ROLL >= -3 && ROLL <= 3)
      {
        FrontLeftMotor.Set(0);
        FrontRightMotor.Set(0);
      }
      if (ROLL >= 3)
      {
        if (YAW <= 3 && YAW >= -3)
        {
          speed = -0.07;
          FrontLeftMotor.Set(0.1);
          FrontRightMotor.Set(-0.09);
        }
        else if (YAW >= 3)
        {
          FrontLeftMotor.Set(0.1 * 0.7);
          FrontRightMotor.Set(-0.09);
        }
        else if (YAW <= -3)
        {
          FrontLeftMotor.Set(0.1);
          FrontRightMotor.Set(-0.09 * 0.7);
        }
      }
      else if (ROLL <= -3)
      {
        if (YAW <= 3 && YAW >= -3)
        {
          FrontLeftMotor.Set(-0.1);
          FrontRightMotor.Set(0.09);
        }
        else if (YAW >= 3)
        {
          FrontLeftMotor.Set(-0.1 * 0.7);
          FrontRightMotor.Set(0.09);
        }
        else if (YAW <= -3)
        {
          FrontLeftMotor.Set(-0.1);
          FrontRightMotor.Set(0.09 * 0.7);
        }
      }
    }
  }
  if (autoChooser == "8")
  {
    if (autoStep == 1)
    {
      Intake.Set(-0.4);
      sleep(1.5);
      Intake.Set(0);
      autoStep++;
      /*if(autoStart == false) {
        driveTimer->Start();
        autoStart = true;
      }
      if(driveTimer->Get() > (units::time::second_t)1.5) {
        autoStep++;
      }*/
    }
    if (autoStep == 2)
    {
      if (YAW <= 173)
      {
        speed = 0.15;
        FrontLeftMotor.Set(speed);
        FrontRightMotor.Set(speed);
      }
      else
      {
        FrontLeftMotor.Set(0);
        FrontRightMotor.Set(0);
        LeftEncoder.SetPosition(0);
        RightEncoder.SetPosition(0);
        autoStep++;
      }
    }
    if (autoStep == 3)
    {
      if (YAW <= 180 && YAW >= 174)
      {
        if (AverageEncoderValue <= 55)
        {
          speed = 0.3;
          FrontRightMotor.Set(speed);
          FrontLeftMotor.Set(-speed);
        }
        else
        {
          FrontRightMotor.Set(0);
          FrontLeftMotor.Set(0);
          autoStep++;
        }
      }
      else
      {
        if (AverageEncoderValue <= 55)
        {
          if (YAW >= 180)
          {
            FrontRightMotor.Set(speed * 0.7);
            FrontLeftMotor.Set(-speed);
          }
          if (YAW <= 174)
          {
            FrontRightMotor.Set(speed);
            FrontLeftMotor.Set(-speed * 0.7);
          }
        }
        else
        {
          FrontRightMotor.Set(0);
          FrontLeftMotor.Set(0);
          Intake.Set(0);
          LeftEncoder.SetPosition(0);
          RightEncoder.SetPosition(0);

          autoStep++;
        }
      }
    }
    if (autoStep == 4)
    {
      IntakePiston.Set(true);
      Intake.Set(0.4);
      sleep(2);
      IntakePiston.Set(false);
      Intake.Set(0);
      autoStep = 5;
    }
    if (autoStep == 5)
    {
      if (YAW >= 7)
      {
        speed = -0.15;
        FrontLeftMotor.Set(speed);
        FrontRightMotor.Set(speed);
      }
      else
      {
        FrontLeftMotor.Set(0);
        FrontRightMotor.Set(0);
        LeftEncoder.SetPosition(0);
        RightEncoder.SetPosition(0);
        autoStep++;
      }
    }
    if (autoStep == 6)
    {
      if (YAW <= 3 && YAW >= -3)
      {
        if (AverageEncoderValue <= 55)
        {
          speed = 0.3;
          FrontRightMotor.Set(speed);
          FrontLeftMotor.Set(-speed);
        }
        else
        {
          FrontRightMotor.Set(0);
          FrontLeftMotor.Set(0);
          autoStep++;
        }
      }
      else
      {
        if (AverageEncoderValue <= 55)
        {
          if (YAW >= 3)
          {
            FrontRightMotor.Set(speed * 0.7);
            FrontLeftMotor.Set(-speed);
          }
          if (YAW <= -3)
          {
            FrontRightMotor.Set(speed);
            FrontLeftMotor.Set(-speed * 0.7);
          }
        }
        else
        {
          FrontRightMotor.Set(0);
          FrontLeftMotor.Set(0);
          // autoStep++;
        }
      }
    }
    if (autoStep == 7)
    {
      Intake.Set(-0.4);
      sleep(1.5);
      Intake.Set(0);
      autoStep++;
    }
  }
}
void Robot::TeleopInit()
{
  timerStarted = false;
  gyro.SetYaw(0);
  gyro.Calibrate();
  clawTimer->Reset();
  Piston.Set(0);
  IntakePiston.Set(false);
  Vent1.Set(0);
  Vent2.Set(0);
  Vent3.Set(0);
  SRX_1.Set(false);
  SRX_2.Set(false);
  SRX_3.Set(false);

  ClawMotor.Set(0);
  ArmOneEncoder.SetPosition(0);
  ArmTwoEncoder.SetPosition(0);
  ExtensionMotorOne.SetSelectedSensorPosition(0);
  ExtensionMotorTwo.SetSelectedSensorPosition(0);

  gyro.Reset();
  // Resetting sensor positions using buttons on the dashboard

  RightEncoder.SetPosition(0);
  LeftEncoder.SetPosition(0);
  gyro.Reset();

  if (m_autonomousCommand != nullptr)
  {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }
}

void Robot::TeleopPeriodic()
{
  frc::SlewRateLimiter<units::scalar> filter{0.1 / 0.5_s};

  buttonValueTwo = frc::SmartDashboard::GetBoolean("DB/Button 1", false);

  frc::SmartDashboard::PutString("DB/String 0", "LD " + std::to_string(LeftEncoderValue));
  frc::SmartDashboard::PutString("DB/String 1", "RD " + std::to_string(RightEncoderValue));
  frc::SmartDashboard::PutString("DB/String 3", "EL " + std::to_string(maxextensionlimit));
  frc::SmartDashboard::PutString("DB/String 4", "EV " + std::to_string(extensionvalue));
  frc::SmartDashboard::PutString("DB/String 5", "AOV " + std::to_string(ArmOneEncoderValue));
  frc::SmartDashboard::PutString("DB/String 6", "ATV " + std::to_string(ArmTwoEncoderValue));
  frc::SmartDashboard::PutString("DB/String 7", "AVDV " + std::to_string(AverageEncoderValue));
  frc::SmartDashboard::PutString("DB/String 8", "ROLL " + ((std::to_string(ROLL))));
  frc::SmartDashboard::PutString("DB/String 9", "YAW " + ((std::to_string(YAW))));

  pcmCompressor.EnableDigital();
  extensionvalue = ExtensionMotorOne.GetSelectedSensorPosition();

  ROLL = gyro.GetRoll() - 2;
  YAW = gyro.GetYaw();

  LeftEncoderValue = -LeftEncoder.GetPosition();
  RightEncoderValue = RightEncoder.GetPosition();
  AverageEncoderValue = (LeftEncoderValue + RightEncoderValue) / 2;

  ArmOneEncoderValue = -ArmOneEncoder.GetPosition();
  ArmTwoEncoderValue = ArmTwoEncoder.GetPosition();
  AverageArmEncoderValue = (ArmTwoEncoderValue + ArmOneEncoderValue) / 2;

  if (buttonValueTwo == true)
  {
    Xbox.SetRumble(frc::GenericHID::RumbleType::kRightRumble, 1.0);
    Xbox.SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 1.0);
    Xbox.SetRumble(frc::Joystick::kRightRumble, 1.0);
    Xbox.SetRumble(frc::Joystick::kLeftRumble, 1.0);
  }
  else if (buttonValueTwo == false)
  {
    Xbox.SetRumble(frc::GenericHID::RumbleType::kRightRumble, 0.0);
    Xbox.SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0.0);
    Xbox.SetRumble(frc::Joystick::kRightRumble, 0.0);
    Xbox.SetRumble(frc::Joystick::kLeftRumble, 0.0);
  }

  // 41 inches
  // 2988.303 is multiplier by inches

  mainlimit = 122520.423;
  // Setting extension limits
  if (AverageArmEncoderValue <= -15)
  {
    // number in parameters + (number in parenthese subtracted from AverageArmValue) must equal division factor
    limitfactor = (AverageArmEncoderValue - 1) / -16;
    maxextensionlimit = mainlimit / limitfactor;
  }
  else
  {
    // resetting limits
    maxextensionlimit = mainlimit;
  }

  // Release the piston while the arm is being extended & retracted
  if (Xbox.GetRawButtonPressed(2) || Xbox.GetRawButtonPressed(1))
  {
    Piston.Set(1);
  }
  // Stops firing piston while arm is extended & retracted
  if (Xbox.GetRawButtonReleased(1) || Xbox.GetRawButtonReleased(2))
  {
    Piston.Set(0);
  }

  if (Xbox.GetRawButton(8))
  {
    // arm to go
    if (extensionvalue <= maxextensionlimit)
    {
      if (extensionvalue <= 75000)
      {
        Piston.Set(1);
        ExtensionMotorOne.Set(0.3);
        ExtensionMotorTwo.Set(0.3);
      }
      else
      {
        ExtensionMotorOne.Set(0);
        ExtensionMotorTwo.Set(0);
        Piston.Set(0);
      }
    }
    else
    {
      ExtensionMotorOne.Set(0);
      ExtensionMotorTwo.Set(0);
    }

    if (AverageArmEncoderValue >= -47)
    {
      if (AverageArmEncoderValue <= -17.5)
      {
        ArmUpOne.Set(-0.25);
        ArmUpTwo.Set(0.25);
      }
      // arm to go down
      else if (AverageArmEncoderValue >= -15.5)
      {
        ArmUpOne.Set(0.25);
        ArmUpTwo.Set(-0.25);
      }
      else
      {
        ArmUpOne.Set(0);
        ArmUpTwo.Set(0);
      }
    }
    else
    {
      ArmUpOne.Set(0);
      ArmUpTwo.Set(0);
    }
  }
  // Between ___ & 41 inches, the arm can retract & extend
  if (!Xbox.GetRawButton(8))
  {
    if (extensionvalue <= maxextensionlimit && extensionvalue >= 0)
    {
      if (Xbox.GetRawButton(1))
      {
        ExtensionMotorOne.Set(0.3);
        ExtensionMotorTwo.Set(0.3);
        currentextend = ExtensionMotorOne.GetSelectedSensorPosition();
      }
      else if (Xbox.GetRawButton(2))
      {
        ExtensionMotorOne.Set(-0.3);
        ExtensionMotorTwo.Set(-0.3);
      }
      else
      {
        ExtensionMotorOne.Set(0);
        ExtensionMotorTwo.Set(0);
        Piston.Set(0);
        currentextend = ExtensionMotorOne.GetSelectedSensorPosition();
      }
    }
    // arm only retract
    if (extensionvalue >= maxextensionlimit)
    {
      if (Xbox.GetRawButton(2))
      {
        ExtensionMotorOne.Set(-0.3);
        ExtensionMotorTwo.Set(-0.3);
        currentextend = ExtensionMotorOne.GetSelectedSensorPosition();
      }
      else
      {
        ExtensionMotorOne.Set(-0.2);
        ExtensionMotorTwo.Set(-0.2);
        Piston.Set(1);
      }
    }
    // arm only extend
    if (extensionvalue <= 0)
    {
      if (Xbox.GetRawButton(1))
      {
        ExtensionMotorOne.Set(0.3);
        ExtensionMotorOne.Set(0.3);
        currentextend = ExtensionMotorOne.GetSelectedSensorPosition();
      }
      else
      {
        ExtensionMotorOne.Set(0);
        ExtensionMotorTwo.Set(0);
        currentextend = ExtensionMotorOne.GetSelectedSensorPosition();
      }
    }

    if ((AverageArmEncoderValue >= -47) && (AverageArmEncoderValue <= 0))
    {
      // autopreset for cube
      if (Xbox.GetRawButton(6))
      {
        ArmUpOne.Set(-0.25);
        ArmUpTwo.Set(0.25);
        currentarm = ArmOneEncoder.GetPosition();
      }
      else if (Xbox.GetRawButton(5))
      {
        ArmUpOne.Set(0.25);
        ArmUpTwo.Set(-0.25);
        currentarm = ArmOneEncoder.GetPosition();
      }
      else
      {
        ArmUpOne.Set(0);
        ArmUpTwo.Set(0);
        currentarm = ArmOneEncoder.GetPosition();
      }
    }

    if (AverageArmEncoderValue <= -47)
    {
      if (Xbox.GetRawButton(6))
      {
        ArmUpOne.Set(-0.2);
        ArmUpTwo.Set(0.2);
        currentarm = ArmOneEncoder.GetPosition();
      }
      else
      {
        ArmUpOne.Set(0);
        ArmUpTwo.Set(0);
        currentarm = ArmOneEncoder.GetPosition();
      }
    }
    if (AverageArmEncoderValue >= 0)
    {
      if (Xbox.GetRawButton(5))
      {
        ArmUpOne.Set(0.25);
        ArmUpTwo.Set(-0.25);
        currentarm = ArmOneEncoder.GetPosition();
      }
      else
      {
        ArmUpOne.Set(0);
        ArmUpTwo.Set(0);
        currentarm = ArmOneEncoder.GetPosition();
      }
    }
  }
  if (Xbox.GetRawButtonPressed(3))
  {
    bothTake = 2;
  }
  if (Xbox.GetRawButtonPressed(4))
  {
    bothTake = 3;
  }

  // Cone intake
  if (bothTake == 1)
  {
    if (Xbox.GetRawAxis(3) >= 0.1)
    {
      coneintake = true;
    }
    if (coneintake == true)
    {
      SRX_1.Set(true);
      SRX_2.Set(true);
      SRX_3.Set(true);
      Lights.Set(true);
      Vent1.Set(false);
      Vent2.Set(false);
      Vent3.Set(false);
    }
    else
    {
      SRX_1.Set(false);
      SRX_2.Set(false);
      SRX_3.Set(false);
      Lights.Set(false);
      Vent1.Set(true);
      Vent1.Set(true);
      Vent1.Set(true);

      ClawMotor.Set(0);
    }
  }

  // turning it on based on the button pressed
  if (bothTake == 2)
  {
    SRX_1.Set(true);
    SRX_2.Set(true);
    SRX_3.Set(true);
    Vent1.Set(false);
    Vent2.Set(false);
    Vent3.Set(false);
    ClawMotor.Set(0.3);
  }
  // toggling the button off based on the button pressed
  else if (bothTake == 3)
  {
    coneintake = false;
    clawTimer->Start();
    if (clawTimer->Get() < (units::time::second_t)3)
    {
      ClawMotor.Set(-0.15);

      SRX_1.Set(false);
      SRX_2.Set(false);
      SRX_3.Set(false);

      Vent1.Set(true);
      Vent2.Set(true);
      Vent3.Set(true);
    }
    else
    {
      Vent1.Set(false);
      Vent2.Set(false);
      Vent3.Set(false);

      bothTake = 1;
      clawTimer->Reset();
    }
  }

  if (JoyStick1.GetRawButton(1))
  {
    Intake.Set(0.5);
    IntakePiston.Set(true);
  }
  else if (JoyStick1.GetRawButton(3))
  {
    Intake.Set(-0.4);
  }
  else
  {
    Intake.Set(0);
    IntakePiston.Set(false);
  }

  // setting base values for teleop
  double WheelX = -Wheel.GetX();
  double JoyY = JoyStick1.GetY();

  /*
  if (JoyY > 0 && !timerStarted) {
    driveTimer->Start();
    timerStarted = true;
  }
  else {
    driveTimer->Reset();
    timerStarted = false;
  }
  units::scalar_t vFactor = driveTimer->Get();

  driveLimit = vFactor *0.05;
  */
  double lmol = ((WheelX * 0.5) + (0.6 * JoyY));
  double rmor = ((WheelX * 0.5) - (0.6 * JoyY)) * 0.9368259;

  FrontLeftMotor.Set(lmol);
  FrontRightMotor.Set(rmor);
}
void Robot::TestPeriodic()
{

  if (Xbox.GetRawButton(4))
  {
    ExtensionMotorOne.Set(0.5);
    ExtensionMotorTwo.Set(0.5);
  }
  else if (Xbox.GetRawButton(1))
  {
    ExtensionMotorOne.Set(-0.5);
    ExtensionMotorTwo.Set(-0.5);
  }
  else
  {
    ExtensionMotorOne.Set(0);
    ExtensionMotorTwo.Set(0);
  }
  AverageArmEncoderValue = (ArmTwoEncoderValue + ArmOneEncoderValue) / 2;
  ArmOneEncoderValue = -ArmOneEncoder.GetPosition();
  ArmTwoEncoderValue = ArmTwoEncoder.GetPosition();

  if (Xbox.GetRawButton(5))
  {
    ArmUpOne.Set(-0.02);
    ArmUpTwo.Set(0.02);
  }
  else if (Xbox.GetRawButton(6))
  {
    ArmUpOne.Set(0.02);
    ArmUpTwo.Set(-0.02);
  }
  else
  {
    ArmUpOne.Set(0);
    ArmUpTwo.Set(0);
  }

  AverageArmEncoderValue = (ArmTwoEncoderValue + ArmOneEncoderValue) / 2;
  ArmOneEncoderValue = -ArmOneEncoder.GetPosition();
  ArmTwoEncoderValue = ArmTwoEncoder.GetPosition();

  buttonValue = frc::SmartDashboard::GetBoolean("DB/Button 0", false);
  buttonValueTwo = frc::SmartDashboard::GetBoolean("DB/Button 1", false);
  buttonValueThree = frc::SmartDashboard::GetBoolean("DB/Button 2", false);
  buttonValueFour = frc::SmartDashboard::GetBoolean("DB/Button 3", false);
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif