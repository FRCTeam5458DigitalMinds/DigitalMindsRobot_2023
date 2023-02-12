// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Libraries
#include "Robot.h"
#include "RobotContainer.h"

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

/*
// PID Controls

// Intake Motors
rev::CANSparkMax IntakeMotorLeft{7, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax IntakeMotorRight{8, rev::CANSparkMax::MotorType::kBrushless};

// Turntable Motor
rev::CANSparkMax TurnTableMotor{9, rev::CANSparkMax::MotorType::kBrushless};

// Extension of Arm
/*TalonFX ExtendMotorOne{1};
TalonFX ExtendMotorTwo{2};

// Arm (Up & Down)

rev::CANSparkMax ArmMotorOne{10, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax ArmMotorTwo{11, rev::CANSparkMax::MotorType::kBrushless};

// You spin me right round baby right round like a record player right round round round
rev::CANSparkMax RotatorMotorOne{12, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax RotatorMotorTwo{13, rev::CANSparkMax::MotorType::kBrushless};

// ClawMotor
rev::CANSparkMax ClawMotor{14, rev::CANSparkMax::MotorType::kBrushless};
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

void Robot::RobotInit()
{

  // camera
  frc::CameraServer::StartAutomaticCapture();

  // Follow ExtendMotorOne
  //ExtendMotorTwo.Follow(ExtendMotorOne);

  // Follow front motors

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
  /* m_autonomousCommand = m_container.GetAutonomousCommand();

   if (m_autonomousCommand != nullptr) {
     m_autonomousCommand->Schedule();
   } */
  clock_begin = steady_clock::now();

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

  if (m_autonomousCommand != nullptr)
  {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }
}

void Robot::TeleopPeriodic()
{
  double JoyY = -Wheel.GetX();
  double WheelX = JoyStick1.GetY();

  if ((WheelX > 0.05 || WheelX < -0.05) && (JoyY > 0.1 || JoyY < -0.1)) {
    FrontLeftMotor.Set((JoyY*0.4) + (0.3*WheelX));
    MiddleLeftMotor.Set((JoyY*0.4) + (0.3*WheelX));
    BackLeftMotor.Set((JoyY*0.4) + (0.3*WheelX));

    FrontRightMotor.Set((JoyY*0.4) - (0.3*WheelX));
    MiddleRightMotor.Set((JoyY*0.4) - (0.3*WheelX));
    BackRightMotor.Set((JoyY*0.4) - (0.3*WheelX));
  } else if (JoyY > 0.1 || JoyY < -0.1) {
    FrontLeftMotor.Set((JoyY * 0.35));
    MiddleLeftMotor.Set((JoyY * 0.35));
    BackLeftMotor.Set((JoyY * 0.35));


    FrontRightMotor.Set((JoyY * 0.35));
    MiddleRightMotor.Set((JoyY * 0.35));
    BackRightMotor.Set((JoyY * 0.35));
  } // Point turning
  else if (JoyStick1.GetRawButton(1)) {
    if (WheelX > 0) {
      FrontRightMotor.Set(-(WheelX * WheelX));
      MiddleRightMotor.Set(-(WheelX * WheelX));
      BackRightMotor.Set(-(WheelX * WheelX));

      FrontLeftMotor.Set((WheelX * WheelX));
      MiddleLeftMotor.Set((WheelX * WheelX));
      BackLeftMotor.Set((WheelX * WheelX));

    } else if (WheelX < 0) {
      FrontRightMotor.Set((WheelX * WheelX));
      MiddleRightMotor.Set((WheelX * WheelX));
      BackRightMotor.Set((WheelX * WheelX));

      FrontLeftMotor.Set(-(WheelX * WheelX));
      MiddleLeftMotor.Set(-(WheelX * WheelX));
      BackLeftMotor.Set(-(WheelX * WheelX));

    }
  } else {
    FrontLeftMotor.Set(0);
    MiddleLeftMotor.Set(0);
    BackLeftMotor.Set(0);

    BackRightMotor.Set(0);
    FrontRightMotor.Set(0);
    MiddleRightMotor.Set(0);
  } 

  /* Joystick and Wheel Teleop code
  double WheelX = Wheel.GetX();
  double JoyY = JoyStick1.GetY();

  if (JoyStick1.GetRawButtonReleased(1))
  { // new method for driving. Library already has controls for west coast drive
    tankDrive.ArcadeDrive(JoyY, WheelX);
  }
  else if (JoyStick1.GetRawButton(1))
  { // start of point turning
    if (WheelX > 0)
    {
      FrontRightMotor.Set(-(WheelX * WheelX));
      FrontLeftMotor.Set((WheelX * WheelX));
    }
    else if (WheelX < 0)
    {
      FrontRightMotor.Set((WheelX * WheelX));
      FrontLeftMotor.Set(-(WheelX * WheelX));
    }
  }
  else
  {
    FrontRightMotor.Set(0);
    FrontLeftMotor.Set(0);
  }
  if (Xbox.GetRawButton(1))
  {
  }

*/
}
void Robot::TestPeriodic()
{
  /*frc::SmartDashboard::PutString("DB/String 0", "This is a string");
  std::string dashData = frc::SmartDashboard::GetString("DB/String 0", "myDefaultData");
  frc::SmartDashboard::PutString("DB/String 1", dashData);
  std::string autoChooser = frc::SmartDashboard::GetString("DB/String 3", "myDefaultData");
  if(autoChooser == "1"){
    frc::SmartDashboard::PutString("DB/String 4", "One");
  }
  else{
    frc::SmartDashboard::PutString("DB/String 4", "Zero")
  } */

  /*SendableChooser<String> auto = new SendableChooser<String>();
   auto.addOption("JustLeaveCommunity", "JustLeaveCommunity");
        auto.addOption("Cit_Circuits", "Cit_Circuits");
        SmartDashboard.putData("Auto Mode", auto);
        String autonMode = auto.getSelected; */

  buttonValue = frc::SmartDashboard::GetBoolean("DB/Button 1", false);
  buttonValueTwo = frc::SmartDashboard::GetBoolean("DB/Button 0", false);
  buttonValueThree = frc::SmartDashboard::GetBoolean("DB/Button 0", false);
  buttonValueFour = frc::SmartDashboard::GetBoolean("DB/Button 0", false);

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
