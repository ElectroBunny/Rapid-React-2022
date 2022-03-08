// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Canenet;


public class CanenetRight extends CommandBase {

  private Canenet canenet;

  public CanenetRight(Canenet inner_canenet) {
    canenet = inner_canenet;
    addRequirements(canenet);

  }


  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    canenet.turnRight(1);
  }

  @Override
  public void end(boolean interrupted) {
    canenet.stopTurning();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
