// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.RobotMap;

public class ControlCollector extends SubsystemBase {
  private WPI_VictorSPX victor_sharsheret = null;

  public ControlCollector() {
    this.victor_sharsheret = new WPI_VictorSPX(RobotMap.VICTOR_SHARSHERET);
    this.victor_sharsheret.setNeutralMode(NeutralMode.Brake);
  }

  public void RiseCollector(double Gain){
    victor_sharsheret.set(ControlMode.PercentOutput, Gain);
  }

  public void DownCollector(double Gain){
    victor_sharsheret.set(ControlMode.PercentOutput, -Gain);
  }

  public void StopControl(){
    victor_sharsheret.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {

  }
}
