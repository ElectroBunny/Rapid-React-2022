// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
public class Shooter extends SubsystemBase {
  public WPI_VictorSPX victor_shooter = null;
  double voltage_to_motor;
  public double startTime;

  public Shooter() {
    this.victor_shooter = new WPI_VictorSPX(RobotMap.VICTOR_SHOOTER);
    this.victor_shooter.setNeutralMode(NeutralMode.Brake);
  }
 
  public void startShoot(double GainShooter){
   victor_shooter.set(ControlMode.PercentOutput, GainShooter); 
  }

  public void stopShoot() {
    victor_shooter.set(ControlMode.PercentOutput, 0); 
  }

  /*
      parms
      H: first high of ball (meters)
      D: first dist of ball (meters)
      h: high of target from the floor (meters)
      d: dist of robot from target (meters)
      alpha: angle of shooting (radiands)
  */
  public double calculateVelocity(double H, double D, double h, double d, double alpha){
    double v0 = (d + D) * Math.sqrt(-5/(h - H - (d + D) * Math.tan(alpha)))/ Math.cos(alpha);
    return v0;
  }
  
  public double getMotorVoltage(){
    return victor_shooter.getMotorOutputPercent();
  }

  @Override
  public void periodic() {
  }
}
