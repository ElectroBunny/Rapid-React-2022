// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Track;

public class RollLeft extends CommandBase {
  private Track m_roller;
  DigitalInput limitSwitch;
  public double startTime, deltaTime;
  
  public RollLeft(Track inner_roller) {
    m_roller = inner_roller;
    limitSwitch = new DigitalInput(0);
    addRequirements(m_roller);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    //m_roller.RollLeft();
  }

  @Override
  public void execute() {
    deltaTime = Timer.getFPGATimestamp() - startTime;
    SmartDashboard.putNumber("Timer Roller Left Up", deltaTime);
    SmartDashboard.putBoolean("limitswitch",  limitSwitch.get());
   
  }

  @Override
  public void end(boolean interrupted) {
//    m_roller.StopRoll();
  }

  @Override
  public boolean isFinished() {
//  return limitSwitch.get();
  return false;
}
}
