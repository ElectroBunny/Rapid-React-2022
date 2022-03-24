// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.NewDriverTrain;
import frc.robot.OI;

public class StartArcadeDrive extends CommandBase {

  private NewDriverTrain driver;
  double forwardY = 0, reverseY = 0, xAxis = 0;

   public StartArcadeDrive(NewDriverTrain innerDriver ) {
    driver = innerDriver;
    addRequirements(driver);
  
  }
  
  @Override
  public void initialize() {

  }

  @Override
  public void execute() {

     forwardY = new OI().getXbox360Joystick().getRightTriggerAxis();
     reverseY = new OI().getXbox360Joystick().getLeftTriggerAxis();
     xAxis = new OI().getXbox360Joystick().getLeftX();
     driver.ArcadeDrive((forwardY - reverseY) , xAxis );

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