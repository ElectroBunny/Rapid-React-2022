// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Subsystems.Shooter;

public class ShootBall extends CommandBase {
  private Shooter balls_shooter;
  public ShootBall(Shooter innerShooter) {
    balls_shooter = innerShooter;
    addRequirements(balls_shooter);

  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    balls_shooter.startShoot(SmartDashboard.getNumber("Shooter Gain", 0));
    SmartDashboard.putNumber("Shooter voltage output ", balls_shooter.getMotorVoltage());
  }

  @Override
  public void end(boolean interrupted) {
    balls_shooter.stopShoot();
  }

  @Override
  public boolean isFinished() {
    return (new OI().getJoystick().getTriggerReleased());
  }
}
