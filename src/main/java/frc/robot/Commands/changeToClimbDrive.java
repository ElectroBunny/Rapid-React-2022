// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.Subsystems.NewDriverTrain;

public class changeToClimbDrive extends CommandBase {
  private NewDriverTrain driver;
  
  public changeToClimbDrive(NewDriverTrain innerDriver) {
    driver = innerDriver;
    addRequirements(driver); 

  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    
    double yAxis = new OI().GetDriverRawAxis(RobotMap.STICK_Y);
    double xAxis = new OI().GetDriverRawAxis(RobotMap.STICK_X);
    driver.ArcadeDrive(yAxis, -xAxis);
    
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
