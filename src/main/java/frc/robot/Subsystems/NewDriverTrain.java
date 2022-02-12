// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.RobotMap;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class NewDriverTrain extends SubsystemBase {
  WPI_TalonSRX m_leftMaster= new WPI_TalonSRX(RobotMap.DRIVE_LEFT_MASTER);
  WPI_TalonSRX m_leftFollower = new WPI_TalonSRX(RobotMap.DRIVE_LEFT_FOLLOWER);
  
  WPI_TalonSRX m_rightMaster = new WPI_TalonSRX(RobotMap.DRIVE_RIGHT_MASTER);
  WPI_TalonSRX m_rightFollower = new WPI_TalonSRX(RobotMap.DRIVE_RIGHT_FOLLOWER);

  DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMaster, m_rightMaster);

  DifferentialDriveKinematics kinematics = new  DifferentialDriveKinematics(0.52);
  DifferentialDriveOdometry  odometry;
  Pose2d m_pose; 
  
  Gyro gyro = new ADXRS450_Gyro(SPI.Port.kMXP);
  // Gyro gyro;
  public NewDriverTrain() {
    // Need to check the starting point 
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), new Pose2d(0, 0, new Rotation2d()));
  
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
    if (Math.abs(forward) < 0.15) { //deadBand
      forward = 0;
    }
    if (Math.abs(turn) < 0.15) {
      turn = 0;
    }
    m_diffDrive.arcadeDrive(forward, turn);

  }
  public double getAngle(){
     return gyro.getAngle();
  }
   
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(gyro.getAngle());
}
  
  public void sendData(){
    SmartDashboard.putNumber("Angle", gyro.getAngle());
  }

  public double Ticks2Meter(double ticks){
    return (ticks/4096) * Math.PI * RobotMap.DIAMETER_WHELL;
     
  }

  @Override
  public void periodic() {

    Rotation2d gyroAngle = Rotation2d.fromDegrees(-gyro.getAngle());
    
    // Update the pose
 //   m_pose = odometry.update(gyroAngle, Ticks2Meter(m_leftMaster.getSelectedSensorPosition()), Ticks2Meter(m_rightMaster.getSelectedSensorPosition()));
//     pose=odometry.update(getHeading(),getSpeeds().leftMetersPerSecond,getSpeeds().rightMetersPerSecond);
 //    SmartDashboard.putNumber("getPose() X:", m_pose.getX());
//     SmartDashboard.putNumber("getPose() Y:", m_pose.getY());
//     SmartDashboard.putNumber("ANGLE", gyro.getAngle());
    // SmartDashboard.putNumber("getSpeeds() leftMetersPerSecond", odometry.getPoseMeters());
     //SmartDashboard.putNumber("getSpeeds() rightMetersPerSecondMetersPerSecond", getSpeeds().rightMetersPerSecond);

  }
}

//    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
/*
 // Drives a specified distance to a specified heading.
  public Command drivePositionGyro(double distanceInches, double heading) {
    return new InstantCommand(() -> currentEncoder = distancesetup(), this).andThen(
      new RunCommand(() -> {
        m_talonsrxright.set(ControlMode.Position, currentEncoder + Units.inchesToMeters(distanceInches) / DriveConstants.kEncoderDistancePerPulse
            , DemandType.AuxPID, heading * 10);
        m_drive.feed();
      }, this).withInterrupt(() -> atSetpoint())
    );
  }

  public void setMaxDriveOutput(Double maxForward, Double maxRotation) {
    m_drive.setMaxOutput(maxForward, maxRotation);
  }

  @Config.ToggleButton
  public void drivePositionGyroTest(boolean enabled) {
    drivePositionGyro(120, getHeading()).schedule();
  }

  @Config.ToggleButton
  public void driveVelocityTest(boolean enabled) {
    velocitysetup();
    new RunCommand(() -> tankDriveVelocity(1, 1), this).withTimeout(5).schedule();
  }
}

  public void drive(ChassisSpeeds speeds, ChassisSpeeds percents) {

    if (speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0 && speeds.omegaRadiansPerSecond == 0) {
      brake();
      return;
    }
 private final DifferentialDriveOdometry m_odometry;
        resetGryo();

        m_odometry = new DifferentialDriveOdometry(getHeading(), AutoConstants.kStartingPosition);
    recordingState.setBoolean(recording);
       updateOdometry();
        x = getPose2d().getTranslation().getX();
        y = getPose2d().getTranslation().getY();

  Rotation2d initialHeading = new Rotation2d(gyro.getAngle());
        pose=new Pose2d(0,0,initialHeading);
        odometry = new DifferentialDriveOdometry(initialHeading, pose);


   

    
    
    public DifferentialDriveWheelSpeeds getSpeeds()
    {
        return new DifferentialDriveWheelSpeeds(
          
            driveLeftMaster.getMotorOutputPercent()*3.0,
            driveRightMaster.getMotorOutputPercent()*3.0
            );
 
    }

public DifferentialDriveKinematics getKinematics (){
        return kinematics;
    }

    public Pose2d getPose(){
        
        return odometry.getPoseMeters();
    }

*/

//