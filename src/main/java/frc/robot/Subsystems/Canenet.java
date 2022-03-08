// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Canenet extends SubsystemBase {
  private WPI_VictorSPX victor_canenet = null;

  public Canenet() {
    this.victor_canenet = new WPI_VictorSPX(RobotMap.VICTOR_CANENET);
    this.victor_canenet.setNeutralMode(NeutralMode.Coast);
  }
  
  public void turnRight(double GainCanenet){
    victor_canenet.set(ControlMode.PercentOutput, GainCanenet); 
  }

  public void turnLeft(double GainCanenet) {
    victor_canenet.set(ControlMode.PercentOutput, -GainCanenet);
  }

  public void stopTurning() {
    victor_canenet.set(ControlMode.PercentOutput, 0); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
