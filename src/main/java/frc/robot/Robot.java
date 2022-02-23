// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.SPI;
public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;
  // Gyro gyro;
  // Gyro gyro1;



  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    SmartDashboard.putNumber("Collector Gain", 0);
    SmartDashboard.putNumber("Shooter Gain", 0);
    SmartDashboard.putNumber("Performance Time", 0);
    // gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
    // gyro1 = new ADXRS450_Gyro(SPI.Port.kMXP);

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
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}
