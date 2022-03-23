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

    // victor_shooter.configNominalOutputForward(0, 100); 
    // victor_shooter.configNominalOutputReverse(0, 100); 
    // victor_shooter.configPeakOutputForward(1, 100); 
    // victor_shooter.configAllowableClosedloopError(0, 0, 100);
    // victor_shooter.configPeakOutputReverse(-1, 100); 
    // victor_shooter.config_kP(0, 1.1, 100); //Proporstional controller
    // victor_shooter.config_kI(0, 0.1, 100); //Integral controller
    // victor_shooter.config_kD(0, 0, 100); //Differential controller
    // victor_shooter.config_kF(0, 0, 100);
  }
 
  public void startShoot(double GainShooter){
   victor_shooter.set(ControlMode.PercentOutput, -GainShooter); 
  }

  public void stopShoot() {
    victor_shooter.set(ControlMode.PercentOutput, 0); 
  }

  public void startReversed(double GainShooter){
    victor_shooter.set(ControlMode.PercentOutput, GainShooter); 
  }


  public double calculateVelocity(double H, double D, double h, double d, double alpha){
    /*
      parms
      H: first high of ball (meters)
      D: first dist of ball (meters)
      h: high of target from the floor (meters)
      d: dist of robot from target (meters)
      alpha: angle of shooting (radiands)
    */
    double v0 = (d + D) * Math.sqrt(-5/(h - H - (d + D) * Math.tan(alpha)))/ Math.cos(alpha);
    return v0;
    // double contDistToSub=0; //update measure start distance from the basket + half of the basket
    // double d = DistanceFromBasket - contDistToSub;
    // double w = 2.72; //check what is w
    // double alpha = d + 0.5 * w;
    // double H=0.23; //HEIGHT of shooter
    // double h = RobotMap.BASKET_HEIGHT - H;
    // double v0 = Math.sqrt((-42.743 * Math.pow(alpha, 2) / (h - 2.747 * alpha)));
  }
  
  public double getMotorVoltage(){
    return victor_shooter.getMotorOutputPercent();
  }
  @Override

  public void periodic() {
    
  }
  }
