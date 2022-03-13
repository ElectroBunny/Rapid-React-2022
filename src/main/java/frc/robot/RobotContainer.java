// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.CollectBalls;
import frc.robot.Commands.ReleaseBalls;
import frc.robot.Commands.ShootBall;
import frc.robot.Commands.StartArcadeDrive;
import frc.robot.Commands.changeToClimbDrive;
import frc.robot.Commands.RollRight;
import frc.robot.Commands.RollLeft;
import frc.robot.Subsystems.NewDriverTrain;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Canenet;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Collector;
import frc.robot.Subsystems.Track;
import frc.robot.Commands.CanenetLeft;
import frc.robot.Commands.CanenetRight;
import frc.robot.Commands.ClimbComb;


public class RobotContainer {
  Timer time = new Timer();
  private final NewDriverTrain driverTrain = new NewDriverTrain();
  private final Shooter ballsShooter = new Shooter();
  private final Track ballsRoller = new Track();
  private final OI m_oi;
  private final Collector ballsCollector = new Collector();
  private final Climber climber = new Climber();
  private final Canenet m_canenet = new Canenet();
  public double startTime;
  double delta_time = 0;
  Shooter victor_shooter = new Shooter();
  double timePeriod = 2;
  public static  Compressor pcmCompressor;

 


  
  public RobotContainer() {
  pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  //pcmCompressor.enableDigital();
  // pcmCompressor.enableAnalog(0, 120);
  // pcmCompressor.enableHybrid(0, 120);
  m_oi = new OI();
  driverTrain.setDefaultCommand(new StartArcadeDrive(driverTrain));
    configureButtonBindings();
  }

  public void onAutoInit(){
    boolean rollermove = true;
    startTime = Timer.getFPGATimestamp();
  
    while (delta_time < 2.5){
      delta_time = Timer.getFPGATimestamp()- startTime;
      driverTrain.ArcadeDrive(-0.58, 0);
    }
    driverTrain.ArcadeDrive(0, 0);

    startTime = Timer.getFPGATimestamp();
    delta_time = 0.0;
    while (rollermove){
      delta_time = Timer.getFPGATimestamp()- startTime;
      ballsShooter.startShoot(1.0);
        if(delta_time > 2)
        ballsRoller.RollLeft(0.30);

        if(delta_time > 2.85){
        ballsRoller.StopRoll();
        rollermove = false;
        }
    }
      ballsShooter.startShoot(0.0);
      ballsRoller.RollRight(0.25);
      startTime = Timer.getFPGATimestamp();
      delta_time = 0.0;
      while (delta_time < 0.8){
        delta_time = Timer.getFPGATimestamp()- startTime;
        }
      ballsRoller.StopRoll();
    }
    
  

  public void onAutoPeriodic(){
  }

  public void onTeleopInit() {
    driverTrain.ArcadeDrive(0, 0);
  }

  public void DataCompressor(){
    boolean enabled = pcmCompressor.enabled();
    boolean pressureSwitch = pcmCompressor.getPressureSwitchValue();
    double current = pcmCompressor.getCurrent();
    double AnalogVoltage =pcmCompressor.getAnalogVoltage();

    SmartDashboard.putNumber("AnalogVoltage", AnalogVoltage);
    SmartDashboard.putNumber("Current", current);
    SmartDashboard.putNumber("Pressure", pcmCompressor.getPressure());
    SmartDashboard.putBoolean("PressureSwitchValue", pressureSwitch);
    SmartDashboard.putBoolean("Commpressor enabled", enabled);
   }
    

  public void onTeleopPeriodic(){
   //  m_oi.buttonsXbox();
 
  DataCompressor();
  }

  public void onDisabledInit(){
    
  }
  public void onDisabledPeriodic(){
    
  }

  public void onTestInit(){
    boolean rollermove=true;
    startTime = Timer.getFPGATimestamp();
  
    while (delta_time < 2.5){
      delta_time = Timer.getFPGATimestamp()- startTime;
      driverTrain.ArcadeDrive(-0.58, 0);
    }
    driverTrain.ArcadeDrive(0, 0);

    startTime = Timer.getFPGATimestamp();
    delta_time = 0.0;
    while (rollermove){
      delta_time = Timer.getFPGATimestamp()- startTime;
      ballsShooter.startShoot(1.0);
        if(delta_time>4)
        ballsRoller.RollLeft(0.20);

        if(delta_time>4.85){
        ballsRoller.StopRoll();
        rollermove=false;
        }
    }
      ballsShooter.startShoot(0.0);
      ballsRoller.RollRight(0.15);
      startTime = Timer.getFPGATimestamp();
      delta_time=0.0;
      while (delta_time<0.6){
      delta_time = Timer.getFPGATimestamp()- startTime;
        }
        ballsRoller.StopRoll();
      
      
  }

  private void configureButtonBindings() {
    m_oi.button1.debounce(0.2, DebounceType.kBoth);
    m_oi.button3.debounce(0.2, DebounceType.kBoth);
    m_oi.button4.debounce(0.2, DebounceType.kBoth);
    m_oi.A.debounce(0.2, DebounceType.kBoth);
    m_oi.B.debounce(0.2, DebounceType.kBoth);
    m_oi.X.debounce(0.2,DebounceType.kBoth);
    m_oi.Y.debounce(0.2,DebounceType.kBoth);


    ((m_oi.button7).and(m_oi.button8)).toggleWhenActive(new changeToClimbDrive(driverTrain));
    m_oi.button5.toggleWhenPressed(new ClimbComb(climber));
    m_oi.povbuttonupxbox.whileHeld(new RollLeft(ballsRoller));
    m_oi.povbuttondownxbox.whileHeld(new RollRight(ballsRoller));

    m_oi.A.whileHeld(new ShootBall(ballsShooter));
    
     m_oi.B.whileHeld(new CollectBalls(ballsCollector));
     m_oi.Y.whileHeld(new ReleaseBalls(ballsCollector));
     m_oi.button4.whileHeld(new CanenetLeft(m_canenet));
     m_oi.button3.whileHeld(new CanenetRight(m_canenet));
    
  }
  
}


