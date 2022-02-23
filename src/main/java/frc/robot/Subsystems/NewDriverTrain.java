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
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class NewDriverTrain extends SubsystemBase {
  WPI_TalonSRX m_leftMaster= new WPI_TalonSRX(RobotMap.DRIVE_LEFT_MASTER);
  WPI_TalonSRX m_leftFollower = new WPI_TalonSRX(RobotMap.DRIVE_LEFT_FOLLOWER);
  
  WPI_TalonSRX m_rightMaster = new WPI_TalonSRX(RobotMap.DRIVE_RIGHT_MASTER);
  WPI_TalonSRX m_rightFollower = new WPI_TalonSRX(RobotMap.DRIVE_RIGHT_FOLLOWER);

  DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMaster, m_rightMaster);

  DifferentialDriveKinematics kinematics = new  DifferentialDriveKinematics(0.52);
  DifferentialDriveOdometry odometry;

  Pose2d m_pose; 

  SlewRateLimiter filter = new SlewRateLimiter(0.5);

  ADXRS450_Gyro gyro;

  public NewDriverTrain() {
    // Need to check the starting point of the robot
    gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

    gyro.calibrate();

   // odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
  
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
    m_diffDrive.arcadeDrive(forward,turn);
    m_diffDrive.feed();
  }

  public void ArcadeDrive2(double forward, double turn,boolean squareInputs){
    if (Math.abs(forward) < 0.15) { //deadBand
      forward = 0;
    }
    if (Math.abs(turn) < 0.15) {
      turn = 0;
    }
    m_diffDrive.arcadeDrive(forward,turn,squareInputs);
    m_diffDrive.feed();
  }

  public void TankDrive(double forward, double turn){
    if (Math.abs(forward) < 0.15) { //deadBand
      forward = 0;
    }
    if (Math.abs(turn) < 0.15) {
      turn = 0;
    }
    m_diffDrive.tankDrive(forward, turn, true);
  }

   public void curvatureDrive(double forward, double turn, boolean squareInputs){
    if (Math.abs(forward) < 0.15) { //deadBand
      forward = 0;
    }
    if (Math.abs(turn) < 0.15) {
      turn = 0;
    }
    m_diffDrive.curvatureDrive(forward,turn,squareInputs);
    m_diffDrive.feed();
  }
  public void voltDrive(){
    m_leftMaster.set(ControlMode.PercentOutput, 0.4);
    m_rightMaster.set(ControlMode.PercentOutput, 0.4);
  }


  //https://wiki.analog.com/first/adxrs450_gyro_board_frc/java

  public void sendData(){
   
    // SmartDashboard.putNumber("Sensor position left", m_leftMaster.getSelectedSensorPosition());
    // SmartDashboard.putNumber("Sensor position right", m_rightMaster.getSelectedSensorPosition());
    // SmartDashboard.putNumber("Sensor velocity left", m_leftMaster.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("Sensor velocity right", m_rightMaster.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("getPose() X:", m_pose.getX());
    // SmartDashboard.putNumber("getPose() Y:", m_pose.getY());
    // SmartDashboard.putNumber("getPose() X with translation:", m_pose.getTranslation().getX());
    // SmartDashboard.putNumber("getPose() Y with translation:", m_pose.getTranslation().getY());
  }

  public double Ticks2Meter(double ticks){
    return (ticks / 4096) * Math.PI * RobotMap.DIAMETER_WHEEL;
  }

  public double getGyroAngle(){
    return gyro.getAngle();

  }
  public double getGyroRate(){
    return gyro.getRate();

  }


  public void GyroToWidget(){
    Shuffleboard.getTab("Gyro WIDGET").add("Gyro",gyro);

  }

  // double error = 90 - gyro.getAngle();

  // // Turns the robot to face the desired direction
  // drive.tankDrive(kP * error, -kP * error);


  @Override
  public void periodic() {
   SmartDashboard.putNumber("Angle From Robot Contianer",gyro.getRate());

    //Rotation2d gyroAngle = Rotation2d.fromDegrees(-gyro.getAngle()); //check
    // Update the pose
   // m_pose = odometry.update(gyroAngle, Ticks2Meter(m_leftMaster.getSelectedSensorPosition()), Ticks2Meter(m_rightMaster.getSelectedSensorPosition()));
    
  }
}



/*


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


  Rotation2d initialHeading = new Rotation2d(gyro.getAngle());
        pose=new Pose2d(0,0,initialHeading);
        odometry = new DifferentialDriveOdometry(initialHeading, pose);


public DifferentialDriveKinematics getKinematics (){
        return kinematics;
    }

    public Pose2d getPose(){
        
        return odometry.getPoseMeters();
    }

*/

//