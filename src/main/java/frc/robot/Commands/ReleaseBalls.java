// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license.
package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Collector;

public class ReleaseBalls extends CommandBase {
  private Collector balls_collector;

  public ReleaseBalls(Collector ballsReleaser) {
    balls_collector = ballsReleaser;
    addRequirements(balls_collector);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    balls_collector.startRelease(0.6);
  }

  @Override
  public void end(boolean interrupted) {
    balls_collector.stopRelease();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
