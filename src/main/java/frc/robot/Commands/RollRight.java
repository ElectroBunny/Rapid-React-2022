// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Track;

public class RollRight extends CommandBase {
  private Track m_roller;
  private  double startTime=0;
  private double delta_time=0;

  public RollRight(Track inner_roller) {
    m_roller = inner_roller;
    addRequirements(m_roller);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    m_roller.RollRight(0.25);
  }

  @Override
  public void execute() {
    delta_time = Timer.getFPGATimestamp()- startTime;
  //  SmartDashboard.putNumber("Timer Roller Left Up", deltaTime);


  }

  @Override
  public void end(boolean interrupted) {
    m_roller.StopRoll();
  }

  @Override
  public boolean isFinished() {
//    return   (delta_time > 0.8);
return false;
  }
}
