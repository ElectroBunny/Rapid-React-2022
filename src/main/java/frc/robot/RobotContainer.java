// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.ClimbDown;
import frc.robot.Commands.ClimbUp;
import frc.robot.Commands.CollectBalls;
import frc.robot.Commands.ReleaseBalls;
import frc.robot.Commands.ShootBall;
import frc.robot.Commands.StartArcadeDrive;
import frc.robot.Commands.changeToClimbDrive;
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
  double delta_time = 0;
  Shooter victor_shooter = new Shooter();
  double timePeriod = 2;
  private Compressor pcmCompressor;
 


  public RobotContainer() {
    pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    m_oi = new OI();
    configureButtonBindings();
    driverTrain.setDefaultCommand(new StartArcadeDrive(driverTrain));
  }

  public void onAutoInit(){
    startTime = Timer.getFPGATimestamp();
  }

  public void onAutoPeriodic(){
    while (delta_time < 3){
      delta_time = Timer.getFPGATimestamp()- startTime;
      driverTrain.ArcadeDrive(-0.42, 0);
    }
    driverTrain.ArcadeDrive(0, 0);
    ballsShooter.startShoot(0.9);
    new edu.wpi.first.wpilibj2.command.WaitCommand(2);
    new RollLeft(ballsRoller);
    new edu.wpi.first.wpilibj2.command.WaitCommand(3);
    ballsRoller.RollRight();
    new edu.wpi.first.wpilibj2.command.WaitCommand(3);
    ballsRoller.StopRoll();


  }

  public void onTeleopInit() {
  //pcmCompressor.enableDigital();
  //  driverTrain.GyroToWidget();
  }

  public void DataCompressor(){
  //  boolean enabled = pcmCompressor.enabled();
  //   boolean pressureSwitch = pcmCompressor.getPressureSwitchValue();
  //   double current = pcmCompressor.getCurrent();
  //   double AnalogVoltage =pcmCompressor.getAnalogVoltage();

  //   SmartDashboard.putNumber("AnalogVoltage", AnalogVoltage);
  //   SmartDashboard.putNumber("Current", current);
  //   SmartDashboard.putNumber("Pressure", pcmCompressor.getPressure());
  //   SmartDashboard.putBoolean("PressureSwitchValue", pressureSwitch);
  //   SmartDashboard.putBoolean("Commpressor enabled", enabled);
   }
    

  public void onTeleopPeriodic(){
    // m_oi.buttonsXbox();
 
  // DataCompressor();
  }

  public void onDisabledInit(){
    
  }
  public void onDisabledPeriodic(){
    
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
    m_oi.button5.whileHeld(new ClimbUp(climber));
    m_oi.button6.whileHeld(new ClimbDown(climber));
    m_oi.button9.whileHeld(new RollLeft(ballsRoller));
    m_oi.button10.whileHeld(new RollRight(ballsRoller));

    m_oi.A.whileHeld(new ShootBall(ballsShooter)
    .alongWith(new SequentialCommandGroup(new edu.wpi.first.wpilibj2.command.WaitCommand(2),
    new RollLeft(ballsRoller)))).whenReleased(new RollRight(ballsRoller).withTimeout(2));
    


     m_oi.B.whileHeld(new CollectBalls(ballsCollector));
     m_oi.Y.whileHeld(new ReleaseBalls(ballsCollector));
    

    
  }
  
}


