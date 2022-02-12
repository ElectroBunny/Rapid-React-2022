// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.RobotMap;

public class Track extends SubsystemBase {
  private WPI_VictorSPX victor_roller = null;

  public Track() {
    this.victor_roller = new WPI_VictorSPX(RobotMap.VICTOR_ROLLER);
    this.victor_roller.setNeutralMode(NeutralMode.Brake);
  }

  public void StartRoll(){
    SmartDashboard.putBoolean("Rolling work started!", !(RobotMap.ShotWorkKey));
    victor_roller.set(ControlMode.PercentOutput, 0.3); 
  }
  public void StopRoll(){
    SmartDashboard.putBoolean("Rolling work stopped!", !(RobotMap.ShotWorkKey));
    victor_roller.set(ControlMode.PercentOutput, 0); 
  }
  @Override
  public void periodic() {
  }
}
