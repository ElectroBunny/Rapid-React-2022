// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.RobotMap;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class NewDriverTrain extends SubsystemBase {
  private WPI_TalonSRX m_leftMaster = new WPI_TalonSRX(RobotMap.DRIVE_LEFT_MASTER);
  private WPI_TalonSRX m_leftFollower = new WPI_TalonSRX(RobotMap.DRIVE_LEFT_FOLLOWER);
  
  private WPI_TalonSRX m_rightMaster = new WPI_TalonSRX(RobotMap.DRIVE_RIGHT_MASTER);
  private WPI_TalonSRX m_rightFollower = new WPI_TalonSRX(RobotMap.DRIVE_RIGHT_FOLLOWER);

  private DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMaster, m_rightMaster);

  DifferentialDriveKinematics kinematics = new  DifferentialDriveKinematics(0.52);
  DifferentialDriveOdometry odometry;

  Pose2d m_pose; 

  private ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

  public NewDriverTrain() {
    gyro.calibrate();

    m_rightMaster.configFactoryDefault();
    m_rightFollower.configFactoryDefault();
    m_leftMaster.configFactoryDefault();
    m_leftFollower.configFactoryDefault();

    m_rightFollower.follow(m_rightMaster);
    m_leftFollower.follow(m_leftMaster);

    m_rightMaster.setInverted(true);
    m_leftMaster.setInverted(false);
    
    m_leftMaster.setNeutralMode(NeutralMode.Brake);
    m_rightMaster.setNeutralMode(NeutralMode.Brake);

    m_rightFollower.setInverted(InvertType.FollowMaster);
    m_leftFollower.setInverted(InvertType.FollowMaster);

    m_leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    m_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

  }

  public void ArcadeDrive(double forward, double turn){
    if (Math.abs(forward) < 0.20) { //deadBand
      forward = 0.0;
    }
    if (Math.abs(turn) < 0.25) {
      turn = 0.0;
    }
    m_diffDrive.arcadeDrive(forward, turn);
  }

  public void changetoCoast(){
  m_leftMaster.setNeutralMode(NeutralMode.Coast);
  m_rightMaster.setNeutralMode(NeutralMode.Coast);
  }
  public void changetoBrake(){
    m_leftMaster.setNeutralMode(NeutralMode.Brake);
    m_rightMaster.setNeutralMode(NeutralMode.Brake);
    }

  public double Ticks2Meter(double ticks){
    return (ticks / 4096) * Math.PI * RobotMap.DIAMETER_WHEEL;
  }

  public ADXRS450_Gyro getGyro(){
    return gyro;
  }

  public void GyroToWidget(){
    Shuffleboard.getTab("Gyro WIDGET").add("Gyro", gyro);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("MotorOutputVoltage LEFT", m_leftMaster.getMotorOutputVoltage());
    SmartDashboard.putNumber("MotorOutputVoltage RIGHT", m_rightMaster.getMotorOutputVoltage());
  }
}