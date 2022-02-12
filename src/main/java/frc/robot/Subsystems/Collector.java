// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Collector extends SubsystemBase {
  private WPI_VictorSPX victor_collector = null;
  public Collector() {
    this.victor_collector = new WPI_VictorSPX(RobotMap.VICTOR_COLLECTOR);
    this.victor_collector.setNeutralMode(NeutralMode.Brake);
  }

  public void startCollect(){
    SmartDashboard.putBoolean("Collecting work started!", !(RobotMap.ShotWorkKey));
    victor_collector.set(ControlMode.PercentOutput, -0.45); 
  }

  public void stopCollect() {
    SmartDashboard.putBoolean("Collecting work stopped!", RobotMap.ShotWorkKey);
    victor_collector.set(ControlMode.PercentOutput, 0); 
  }

  public void startRelease(){
    SmartDashboard.putBoolean("Releasing work started!", !(RobotMap.ShotWorkKey));
    victor_collector.set(ControlMode.PercentOutput, 0.45); 
  }

  public void stopRelease() {
    SmartDashboard.putBoolean("Releasing work stopped!", RobotMap.ShotWorkKey);
    victor_collector.set(ControlMode.PercentOutput, 0); 
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
