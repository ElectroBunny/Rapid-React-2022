// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.NewDriverTrain;

import frc.robot.OI;
import frc.robot.RobotMap;

public class StartArcadeDrive2 extends CommandBase {
  // private NewDriverTrain driver;
  private NewDriverTrain driver;

  public StartArcadeDrive2(NewDriverTrain innerDriver ) {
    driver = innerDriver;
    addRequirements(driver);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double xAxis = new OI().GetDriverRawAxis(RobotMap.STICK_Y);
    double yAxis = new OI().GetDriverRawAxis(RobotMap.STICK_X);
    driver.ArcadeDrive2(-xAxis, yAxis, true);
  }

  @Override
  public void end(boolean interrupted) {
    driver.ArcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
