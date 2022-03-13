// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Climber;


public class ClimbComb extends CommandBase {
  private Climber climber;
  private int subChanger = 3;

  public ClimbComb(Climber innerClimber) {
    climber = innerClimber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (subChanger % 3 == 0){
      climber.offClimb();
    }
    else if(subChanger % 3 == 1){
      climber.openClimb();
    }
    else{ // if(subChanger % 3 == 2) - efficiency
      climber.closeClimb();
    }
  }

@Override
public void end(boolean interrupted) {
  subChanger += 1;
  SmartDashboard.putNumber("subChanger", subChanger);
}

@Override
public boolean isFinished() {
  return false;

}
}

