// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/Timer.h>//added
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <rev/SparkMax.h>
#include "rev/SparkMax.h"

#include <frc/AddressableLED.h>

#include <frc/LEDPattern.h>
//rev::spark::SparkMax m_leftFrontMotor{2, rev::spark::SparkMax::MotorType::kBrushless};

/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with tank steering.
 */

using namespace frc;
using namespace std;

class Robot : public frc::TimedRobot {
  private:
  //static constexpr int kLength = 60;

  // PWM port 0
  // Must be a PWM header, not MXP or DIO
  //frc::AddressableLED m_led{0};
  //std::array<frc::AddressableLED::LEDData, kLength>
  //    m_ledBuffer;  // Reuse the buffer
  

  // Our LED strip has a density of 120 LEDs per meter


  //using namespace rev::spark;

  //frc::PWMSparkMax m_leftFrontMotor{1};//front
  rev::spark::SparkMax m_leftFrontMotor {1, rev::spark::SparkMax::MotorType::kBrushless};

  //frc::PWMSparkMax m_leftBackMotor{2};//back
  rev::spark::SparkMax m_leftBackMotor {2, rev::spark::SparkMax::MotorType::kBrushless};

  //frc::PWMSparkMax m_rightFrontMotor{3};
  rev::spark::SparkMax m_rightFrontMotor {3, rev::spark::SparkMax::MotorType::kBrushless};

  //frc::PWMSparkMax m_rightBackMotor{4};
  rev::spark::SparkMax m_rightBackMotor {4, rev::spark::SparkMax::MotorType::kBrushless};

// Creating my odometry object. Here,
// our starting pose is 5 meters along the long end of the field and in the
// center of the field along the short end, facing forward.

// frc::DifferentialDriveOdometry m_odometry{
//   m_gyro.GetRotation2d(),
//   units::meter_t{m_leftEncoder.GetDistance()},
//   units::meter_t{m_rightEncoder.GetDistance()},
//   frc::Pose2d{5_m, 13.5_m, 0_rad}};

  frc::DifferentialDrive m_robotDrive{
      [&](double output) { m_leftFrontMotor.Set(output); },
      [&](double output) { m_rightFrontMotor.Set(output); }};
  frc::Joystick m_leftStick{0};
  frc::Joystick m_rightStick{1};
  frc::Timer m_timer;//added

  // Our LED strip has a density of 120 LEDs per meter
    //units::meter_t kLedSpacing{1 / 120.0};

    // Create an LED pattern that will display a rainbow across
    // all hues at maximum saturation and half brightness
    //frc::LEDPattern m_rainbow = frc::LEDPattern::Rainbow(255, 128);

    // Create a new pattern that scrolls the rainbow pattern across the LED
    // strip, moving at a speed of 1 meter per second.
    //frc::LEDPattern m_scrollingRainbow =
        //m_rainbow.ScrollAtAbsoluteSpeed(1_mps, kLedSpacing);

 public:
    static constexpr int kLength = 30;
    frc::AddressableLED m_led{0};

    array<AddressableLED::LEDData, kLength> b_ledBuffer;
    array<AddressableLED::LEDData, kLength> p_ledBuffer;
    array<AddressableLED::LEDData, kLength> o_ledBuffer;
    array<AddressableLED::LEDData, kLength> m_ledBuffer;

    int firstPixelHue = 0;
    bool color = true;
    bool rainbowing = false;

    int colorState = 0;
    bool LcolorPressed = false;
    bool RcolorPressed = false;
    float rampRatePercent = 0.5;
    int state = 1;
  void RobotInit() override {
    

    m_led.SetLength(kLength);
    m_led.SetData(b_ledBuffer);
    m_led.Start();

    for (int i=0; i<kLength; i++) {
      p_ledBuffer[i].SetRGB(85, 19, 135); // purple
    }

    for (int i=0; i<kLength; i++) {
      o_ledBuffer[i].SetRGB(192, 90, 22); // orange
    }

    for (int i=0; i<kLength; i++) {
      b_ledBuffer[i].SetRGB(0, 0, 0); // blank
    }
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    //m_led.SetLength(kLength);
    //m_led.SetData(m_ledBuffer);
    //m_led.Start();


    wpi::SendableRegistry::AddChild(&m_robotDrive, &m_leftFrontMotor);
    wpi::SendableRegistry::AddChild(&m_robotDrive, &m_rightFrontMotor);

    //m_leftBackMotor.IsFollower();
    //m_rightFrontMotor.AddFollower(m_rightBackMotor);

    //m_rightFrontMotor.setOpenLoopRampRate(2.0);
    //m_leftFrontMotor.setOpenLoopRampRate(2.0);
    //m_rightBackMotor.setOpenLoopRampRate(2.0);
    //m_leftBackMotor.setOpenLoopRampRate(2.0);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightFrontMotor.SetInverted(true);
    m_rightBackMotor.SetInverted(true);

    //for auto
    m_robotDrive.SetExpiration(100_ms);
    m_timer.Start();
  }
  
//added
  void AutonomousInit() override { m_timer.Restart(); }

  void AutonomousPeriodic() override {
    // Drive for 2 seconds
    if (m_timer.Get() < 2_s) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_robotDrive.ArcadeDrive(0.5, 0.0, false);
    } else {
      // Stop robot
      m_robotDrive.ArcadeDrive(0.0, 0.0, false);
    }
  }

  void TeleopInit() override {
  }

//added
void DriveManager(int driveButton, int speedIncButton, int speedDecButton)
{
  if(state>3)
  {
    state = 1;
  }

   //This button changes the state of driving
  if (m_leftStick.GetRawButtonReleased(driveButton)) {
   state+=1;
  }
 
 //this button increases the speed by 5%
  if (m_leftStick.GetRawButtonReleased(speedIncButton)) {
   rampRatePercent+=0.05;
  }

  //this button decreases the speed by 5%
  if (m_leftStick.GetRawButtonReleased(speedDecButton)) {
   rampRatePercent-=0.05; 
  }

  // Drive with tank style
  if(state == 1)
  {
    m_robotDrive.TankDrive(-m_leftStick.GetRawAxis(1)*rampRatePercent, -m_leftStick.GetRawAxis(5)*rampRatePercent);
  }
  else if(state == 2)
  {
    //Arcade drive 
    m_robotDrive.ArcadeDrive(-m_leftStick.GetY()*rampRatePercent, -m_leftStick.GetX()*rampRatePercent);
  }
  else if(state == 3)
  {
    //Arcade drive dual
    m_robotDrive.ArcadeDrive(-m_leftStick.GetY()*rampRatePercent, -m_leftStick.GetRawAxis(4)*rampRatePercent);
  }

  
    //Set back motors to follow front
    m_rightBackMotor.Set(m_rightFrontMotor.Get());
    m_leftBackMotor.Set(m_leftFrontMotor.Get());

}
void rainbow() {
    for (int i = 0; i < kLength; i++) {
      const auto pixelHue = (firstPixelHue+(i*180/kLength))%180;
      m_ledBuffer[i].SetHSV(pixelHue, 255, 128);
    }
    firstPixelHue+=3;
    firstPixelHue %= 180;
  }

  void TeleopPeriodic() override {
    // Run the rainbow pattern and apply it to the buffer
    //m_scrollingRainbow.ApplyTo(m_ledBuffer);
    // Set the LEDs
    //m_led.SetData(m_ledBuffer);

    //Fucntion to set speed multiplyer and drive mode. See Void DriveManager for more details 
              //drive button, speedincbutton, speeddecbutton
    DriveManager(4, 2, 3);
    //m_robotDrive.TankDrive(-m_leftStick.GetRawAxis(1)*0.5, -m_leftStick.GetRawAxis(5)*0.5);

    //m_rightFrontMotor.Set(0.05);
    //m_leftFrontMotor.Set(0.05);
    //m_rightBackMotor.Set(0.05);
    //m_leftBackMotor.Set(0.05);
    //m_robotDrive.TankDrive(0.05, 0.05);
    //m_robotDrive.ArcadeDrive(-m_leftStick.GetY()*0.2, m_leftStick.GetX()*0.2);
     if (m_leftStick.GetRawAxis(2) > 0.5 && m_leftStick.GetRawAxis(3) > 0.5) {
        rainbowing = true;
      } else {
        rainbowing = false;
      }
    if (rampRatePercent >= 0.5) {
        rainbowing = true;
      } else {
        rainbowing = false;
      }


      if (rainbowing) {
        rainbow();
        m_led.SetData(m_ledBuffer);
      } else {
        if (colorState == 0) {
          m_led.SetData(b_ledBuffer);
        } else if (colorState == 1) {
          m_led.SetData(o_ledBuffer);
        } else if (colorState == 2) {
          m_led.SetData(p_ledBuffer);
        } 
      }
  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
