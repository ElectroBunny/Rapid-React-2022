// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;
public class Shooter extends SubsystemBase {
  private WPI_VictorSPX victor_shooter = null;
  double voltage_to_motor;
  public double startTime;
  double kP = 0;
  double deltaTime = 0;
  
  public Shooter() {
    Timer time = new Timer();
    this.victor_shooter = new WPI_VictorSPX(RobotMap.VICTOR_SHOOTER);
    this.victor_shooter.setNeutralMode(NeutralMode.Brake);
    victor_shooter.configNominalOutputForward(0, 100); 
    victor_shooter.configNominalOutputReverse(0, 100); 
    victor_shooter.configPeakOutputForward(1, 100); 
    victor_shooter.configAllowableClosedloopError(0, 0, 100);
    victor_shooter.configPeakOutputReverse(-1, 100); 
    victor_shooter.config_kP(0, 1.5, 100);
    victor_shooter.config_kI(0, 0.1, 100);
    victor_shooter.config_kD(0, 0, 100);
    victor_shooter.config_kF(0, 0, 100);


  }
 
  public void startShoot(double GainShooter){
   victor_shooter.set(ControlMode.PercentOutput, -GainShooter); 
  }


  public void stopShoot() {
    victor_shooter.set(ControlMode.PercentOutput, 0); 
  }
  public double calculateVoltage(double DistanceFromBasket){
    double contDistToSub=0; //update measure start distance from the basket + half of the basket
    double d = DistanceFromBasket - contDistToSub;
    double w = 2.72; //check what is w
    double alpha = d + 0.5 * w;
    double H=0.7; //HEIGHT of shooter
    double h = RobotMap.BASKET_HEIGHT - H;
    double v0 = Math.sqrt((-42.743 * Math.pow(alpha, 2) / (h - 2.747 * alpha)));
    return v0;
  }
  public double getMotorVoltage(){
    return victor_shooter.getMotorOutputPercent();
  }
  @Override

  public void periodic() {
   voltage_to_motor = SmartDashboard.getNumber("Enter the voltage to the shooter", -1);
  }

  public void startShootTime(double GainShooter, double Performance_time){
    startTime = Timer.getFPGATimestamp();
    // victor_shooter.set(ControlMode.PercentOutput, -GainShooter); 
    while (deltaTime < Performance_time){
      deltaTime = Timer.getFPGATimestamp()- startTime;
      kP = (deltaTime / Performance_time);
      victor_shooter.set(ControlMode.PercentOutput, kP * GainShooter);
    }
  }


}
