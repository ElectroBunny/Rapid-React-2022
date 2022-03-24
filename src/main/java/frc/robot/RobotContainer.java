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
import frc.robot.Commands.DownCollector;
import frc.robot.Commands.ReleaseBalls;
import frc.robot.Commands.ShootBall;
import frc.robot.Commands.ReversedShoot;
import frc.robot.Commands.StartArcadeDrive;
import frc.robot.Commands.UpCollector;
import frc.robot.Commands.changeToClimbDrive;
import frc.robot.Subsystems.NewDriverTrain;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Canenet;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Collector;
import frc.robot.Subsystems.ControlCollector;
import frc.robot.Commands.CanenetLeft;
import frc.robot.Commands.CanenetRight;
import frc.robot.Commands.ClimbComb;


public class RobotContainer {
  private final NewDriverTrain driverTrain = new NewDriverTrain();
  private final Shooter ballsShooter = new Shooter();
  private final OI m_oi;
  private final Collector ballsCollector = new Collector();
  private final Climber climber = new Climber();
  private final Canenet m_canenet = new Canenet();
  private final ControlCollector collectorController = new ControlCollector();
  public static Compressor pcmCompressor;
  public double startTime;
  public double delta_time = 0;

  public RobotContainer() {
  pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  // pcmCompressor.enableDigital();
  // pcmCompressor.enableAnalog(-120, 120);
  // pcmCompressor.enableHybrid(-120, 120);
  m_oi = new OI();
  driverTrain.setDefaultCommand(new StartArcadeDrive(driverTrain));
  configureButtonBindings();
  }

  public void onAutoInit(){
    //Shoots to the lower hub in the autonomus mode
    startTime = Timer.getFPGATimestamp();
    while (delta_time < 5){
      delta_time = Timer.getFPGATimestamp() - startTime;
      ballsShooter.startShoot(1.0);
    }
    ballsShooter.stopShoot();
    startTime = 0;
    delta_time = 0;

    //Drives to the outside of the tarmac in the autonomus mode
    startTime = Timer.getFPGATimestamp();
    while (delta_time < 2){
      delta_time = Timer.getFPGATimestamp() - startTime;
      driverTrain.ArcadeDrive(0.7, 0);
    }
    driverTrain.ArcadeDrive(0, 0);

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
   // DataCompressor();
  }

  public void onDisabledInit(){
    
  }
  public void onDisabledPeriodic(){
    
  }

  public void onTestInit(){
      
  }

  private void configureButtonBindings() {
    m_oi.button1.debounce(0.2, DebounceType.kBoth);
    m_oi.button3.debounce(0.2, DebounceType.kBoth);
    m_oi.button4.debounce(0.2, DebounceType.kBoth);
    m_oi.A.debounce(0.2, DebounceType.kBoth);
    m_oi.B.debounce(0.2, DebounceType.kBoth);
    m_oi.X.debounce(0.2,DebounceType.kBoth);
    m_oi.Y.debounce(0.2,DebounceType.kBoth);


    m_oi.button5.whileHeld(new ClimbComb(climber));

    m_oi.A.whileHeld(new ShootBall(ballsShooter));
    m_oi.X.whileHeld(new ReversedShoot(ballsShooter));
    
    //  m_oi.B.whileHeld(new CollectBalls(ballsCollector));
    //  m_oi.Y.whileHeld(new ReleaseBalls(ballsCollector));
    m_oi.button8.whileHeld(new CollectBalls(ballsCollector));
    m_oi.button7.whileHeld(new ReleaseBalls(ballsCollector));

     m_oi.button4.whileHeld(new CanenetLeft(m_canenet));
     m_oi.button3.whileHeld(new CanenetRight(m_canenet));

    //  m_oi.xbox5.whileHeld(new DownCollector(collectorController));
    //  m_oi.xbox6.whileHeld(new UpCollector(collectorController));
    m_oi.button10.whileHeld(new DownCollector(collectorController));
    m_oi.button9.whileHeld(new UpCollector(collectorController));
  }
}


