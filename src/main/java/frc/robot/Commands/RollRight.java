// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Track;
import frc.robot.OI;

public class RollRight extends CommandBase {
  private Track m_roller;

  public RollRight(Track inner_roller) {
    m_roller = inner_roller;
    addRequirements(m_roller);
  }

  @Override
  public void initialize() {
    m_roller.RollRight();
  }

  @Override
  public void execute() {
    //m_roller.RollRight();

  }

  @Override
  public void end(boolean interrupted) {
    m_roller.StopRoll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  //  return (new OI().getJoystick().getRawButtonPressed(6));
  }
}
