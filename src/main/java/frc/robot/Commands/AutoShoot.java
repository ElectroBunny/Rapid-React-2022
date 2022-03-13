// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Track;

public class AutoShoot extends CommandBase {
  private Track m_track;
  private Shooter m_shotter;
  private double startTime=0, deltaTime=0, startTimeEnd=0, deltaTimeEnd=0;
  private boolean rollermove=true;

  /** Creates a new AutoShoot. */
  public AutoShoot(Shooter inner_shotter, Track inner_track) {

    m_shotter = inner_shotter;
    m_track = inner_track;
      addRequirements(m_shotter,m_track);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shotter.startShoot(1.0);
    deltaTime = Timer.getFPGATimestamp() - startTime;

    if(deltaTime>1.0)
      m_track.RollLeft(0.27);
    
    if(deltaTime>1.85)
      m_track.RollLeft(0.0);

    SmartDashboard.putNumber("Timer AutoShoot", deltaTime);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shotter.startShoot(0.0);
    startTimeEnd=Timer.getFPGATimestamp();
    m_track.RollRight(0.25);

    while(rollermove){
      deltaTimeEnd = Timer.getFPGATimestamp() - startTimeEnd;
      
      if(deltaTimeEnd>0.80){
      m_track.RollRight(0.0);
      rollermove=false;
      }
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
