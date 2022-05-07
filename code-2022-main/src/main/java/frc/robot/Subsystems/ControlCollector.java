// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.RobotMap;

// Control on the arm for collecting the balls
public class ControlCollector extends SubsystemBase {
  private WPI_VictorSPX victor_arm = null;

  public ControlCollector() {
    this.victor_arm = new WPI_VictorSPX(RobotMap.VICTOR_ARM);
    this.victor_arm.setNeutralMode(NeutralMode.Brake);
  }

  public void changeDirection(double Gain){
    victor_arm.set(ControlMode.PercentOutput, Gain);
  }

  public void StopControl(){
    victor_arm.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
  }
}
