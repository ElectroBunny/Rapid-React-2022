// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Subsystems.NewDriverTrain;

public class XboxDrive extends CommandBase {
  private NewDriverTrain driver;
  double turningValue = 0;
  boolean once = false;
  boolean drivewithfix = true;
  double currnetAngle = 0;
  
  /** Creates a new XboxDrive. */
  public XboxDrive(NewDriverTrain innerDriver) {
    driver = innerDriver;
    addRequirements(driver); 

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      
    double yAxis = new OI().GetXboxLEFTYRawAxis();
    double xAxis = new OI().GetXboxRIGHTXRawAxis();
    driver.ArcadeDrive(yAxis, xAxis);
    
   SmartDashboard.putNumber("xAxis", xAxis);
   SmartDashboard.putNumber("yAxis", yAxis);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driver.ArcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
