// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ControlCollector;

public class UpCollector extends CommandBase {
  private ControlCollector collector_controller;

  public UpCollector(ControlCollector innerController) {
    collector_controller = innerController;
    addRequirements(collector_controller);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    collector_controller.RiseCollector(0.55);

  }

  @Override
  public void end(boolean interrupted) {
    collector_controller.StopControl();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
