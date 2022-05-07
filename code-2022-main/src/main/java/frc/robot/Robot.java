// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {

  CameraServer.startAutomaticCapture();
  // Camera for checking the posiotion for climb (CHECK??)
  CvSink cvSink= CameraServer.getVideo();
  CvSource outputStream = CameraServer.putVideo("Blur", 640, 480);
  m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }


  @Override
  public void autonomousInit() {
     m_robotContainer.onAutoInit();
  }

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    m_robotContainer.onTeleopInit();
  }

  @Override
  public void teleopPeriodic() {
    m_robotContainer.onTeleopPeriodic();
  }

  @Override
  public void disabledInit() {
    m_robotContainer.onDisabledInit();
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.onDisabledPeriodic();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
}
