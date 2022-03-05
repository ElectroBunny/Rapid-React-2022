// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Commands.ClimbDown;
import frc.robot.Commands.ClimbUp;
import frc.robot.Commands.CollectBalls;
import frc.robot.Commands.ReleaseBalls;
import frc.robot.Commands.ShootBall;
import frc.robot.Commands.StartArcadeDrive;
import frc.robot.Commands.XboxDrive;
import frc.robot.Commands.loadAndShoot;
import frc.robot.Commands.RollRight;
import frc.robot.Commands.RollLeft;
import frc.robot.Subsystems.NewDriverTrain;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Collector;
import frc.robot.Subsystems.Track;


public class RobotContainer {
  Timer time = new Timer();
  private final NewDriverTrain driverTrain = new NewDriverTrain();
  private final Shooter ballsShooter = new Shooter();
  private final Track ballsRoller = new Track();
  private final OI m_oi;
  private final Collector ballsCollector = new Collector();
  private final Climber climber = new Climber();
  public double startTime;
  double kP = 0; 
  double delta_time = 0;
  Shooter victor_shooter = new Shooter();
  double timePeriod = 2;
  Compressor pcmCompressor;
  public RobotContainer() {
    driverTrain.getGyro().calibrate();
    m_oi = new OI();
    configureButtonBindings();
    driverTrain.setDefaultCommand(new StartArcadeDrive(driverTrain));
    pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    pcmCompressor.enableAnalog(0, 120);
  }
  


  public void onAutoInit(){
    startTime = Timer.getFPGATimestamp();
    
  }

  public void onAutoPeriodic(){
    while (delta_time < 5){
      delta_time = Timer.getFPGATimestamp()- startTime;
      driverTrain.ArcadeDrive(-0.4, 0);
    }
    driverTrain.ArcadeDrive(0, 0);

    
  }

  public void onTeleopInit() {

    driverTrain.GyroToWidget();
    ballsRoller.resetBlock();

  }

    

  public void onTeleopPeriodic(){

  }

  public void onDisabledInit(){
    driverTrain.changetoCoast();
    
  }
  public void onDisabledPeriodic(){
    driverTrain.changetoCoast();
    driverTrain.getGyro().calibrate();
    
  }

  private void configureButtonBindings() {
    m_oi.button1.debounce(0.1, DebounceType.kBoth);
    m_oi.button3.debounce(0.1, DebounceType.kBoth);
    m_oi.button4.debounce(0.1, DebounceType.kBoth);
    // m_oi.button1.whileHeld(new ShootBall(ballsShooter));
    m_oi.button1.whileHeld(new loadAndShoot(ballsShooter, ballsRoller)); //check
    m_oi.button3.whileHeld(new CollectBalls(ballsCollector));
    m_oi.button4.whileHeld(new ReleaseBalls(ballsCollector));
    // m_oi.button5.whileHeld(new RollLeft(ballsRoller));
    // m_oi.button6.whileHeld(new RollRight(ballsRoller));
    ((m_oi.button7).and(m_oi.button8)).whenActive(new XboxDrive(driverTrain));
    m_oi.L1.whileHeld(new ClimbUp(climber));
    m_oi.R1.whileHeld(new ClimbDown(climber));
  }
}

