// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.RobotMap;

public class Track extends SubsystemBase {
  private WPI_VictorSPX victor_roller = null;
  // private Spark victor_roller = null;

  public Track() {
    this.victor_roller = new WPI_VictorSPX(RobotMap.VICTOR_ROLLER);
    this.victor_roller.setNeutralMode(NeutralMode.Brake);
  }

  public void resetBlock(){
    victor_roller.set(ControlMode.PercentOutput, 0.4);
    new WaitCommand(2);
    victor_roller.set(ControlMode.PercentOutput, 0);
  }

  public void RollLeft() {
    victor_roller.set(ControlMode.PercentOutput, -0.4);
  }
  
  public void RollRight() {
    victor_roller.set(ControlMode.PercentOutput, 0.4);
  }

  public void StopRoll() {
    victor_roller.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
  }
}
