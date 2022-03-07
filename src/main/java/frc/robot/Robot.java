// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;


  @Override
  public void robotInit() {
//✡🕍
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
    m_robotContainer.onAutoPeriodic();

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
