// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.OI;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Track;

public class loadAndShoot extends CommandBase {
  private Shooter balls_shooter;
  private Track balls_roller;
  public double startTime, deltaTime;
  boolean timerOver = true;

  public loadAndShoot(Shooter innerShooter, Track innerRoller) {
    this.balls_shooter = innerShooter;
    this.balls_roller = innerRoller;
    addRequirements(balls_shooter, balls_roller);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    balls_shooter.startShoot(SmartDashboard.getNumber("Shooter Gain", 0));
    SmartDashboard.putNumber("Shooter voltage output ", balls_shooter.getMotorVoltage());
    
    while (timerOver){
      SmartDashboard.putNumber("DelaTime ", deltaTime);
      deltaTime = Timer.getFPGATimestamp() - startTime;
      if(deltaTime > 2){
        balls_roller.RollLeft(); // need to check the direcation 
        timerOver = false;
    }
  }
  
  }

  @Override
  public void end(boolean interrupted) {
    balls_shooter.stopShoot();
    balls_roller.StopRoll();
    balls_roller.RollRight();
    new WaitCommand(1);
    balls_roller.StopRoll();
  }

  @Override
  public boolean isFinished() {
    return (new OI().getJoystick().getTriggerReleased());
  }
}
