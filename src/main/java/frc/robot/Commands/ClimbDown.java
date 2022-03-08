// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Subsystems.Climber;


public class ClimbDown extends CommandBase {
  private Climber climber;

  public ClimbDown(Climber innerClimber) {
    climber = innerClimber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.closeClimb();
  }

@Override
public void end(boolean interrupted) {
  climber.offClimb();
}

@Override
public boolean isFinished() {
     return false;

}
}


