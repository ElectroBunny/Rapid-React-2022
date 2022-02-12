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
public class Shooter extends SubsystemBase {
  private WPI_VictorSPX victor_shooter = null;
  public static double voltage_to_motor = -1; 
  public Shooter() {
    this.victor_shooter = new WPI_VictorSPX(RobotMap.VICTOR_SHOOTER);
    this.victor_shooter.setNeutralMode(NeutralMode.Brake);
  }

  public void startShoot(){
    SmartDashboard.putBoolean("Shooting work started!", !(RobotMap.ShotWorkKey));
    victor_shooter.set(ControlMode.PercentOutput, voltage_to_motor); 
  }

  public void stopShoot() {
    SmartDashboard.putBoolean("Shoot work stopped!", RobotMap.ShotWorkKey);
    victor_shooter.set(ControlMode.PercentOutput, 0); 
  }
  @Override

  public void periodic() {
    ;
  }
}
