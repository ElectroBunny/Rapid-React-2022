// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.CollectBalls;
import frc.robot.Commands.ReleaseBalls;
import frc.robot.Commands.ShootBall;
import frc.robot.Commands.StartArcadeDrive;
import frc.robot.Commands.StartArcadeDrive2;
import frc.robot.Commands.StartCurvatureDrive;
import frc.robot.Commands.StartTankDrive;
import frc.robot.Commands.RollerWork;
import frc.robot.Subsystems.NewDriverTrain;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Collector;
import frc.robot.Subsystems.Track;


public class RobotContainer {
  Timer time = new Timer();
  private final NewDriverTrain driverTrain = new NewDriverTrain();
  private final Shooter ballsShooter = new Shooter();
  private final Track ballsRoller = new Track();
  private final OI m_oi;
  private final Collector ballsCollector = new Collector();
  public double startTime;
  double kP = 0; 
  double delta_time = 0;
  Shooter victor_shooter = new Shooter();
  
 // private final SendableChooser<drivingType> m_driveChooser;

  // public enum drivingType{
  //   ArcadeDrive,
  //   ArcadeDrive2,
  //   CurvatureDrive,
  //   TankDrive;

    
  // }

    // private AutoDriveCommand autoDrive; 
    // private BullCommand surpriseCommand;
    //  Command driveCommandarcadeDrive = new Command() {
    //  driverTrain.GetDriverRawAxis(RobotMap.STICK_Y)  
    //  }

    //     Command arcadeDriveCommand = new ArcadeDriveCommand(
    //         robotDrive, 
    //         () -> -controller.getY(), 
    //         () -> controller.getX(), 
    //         () -> controller.getRawButton(Constants.Controller.leftBumper));
    // Command shootCommand = new ShootCommand(
    //         shootingSubsystem,
    //         () -> controller.getRawButton(Constants.Controller.leftTrigger),
    //         () -> controller.getRawButton(Constants.Controller.rightTrigger),          
    //         () -> controller.getRawAxis(3));       
    //     shootingSubsystem.setDefaultCommand(shootCommand);

    //     autoDrive = new AutoDriveCommand(robotDrive);
    //     surpriseCommand = new BullCommand(robotDrive);
        
    //     robotDrive.setDefaultCommand(arcadeDriveCommand);

    //     autoChooser.setDefaultOption("Default Auto", autoDrive);
    //     autoChooser.addOption("Surprise Auto", surpriseCommand);
    //     SmartDashboard.putData(autoChooser);

  public RobotContainer() {
    m_oi = new OI();
    configureButtonBindings();
    driverTrain.setDefaultCommand(new StartArcadeDrive(driverTrain));
    // m_driveChooser = new SendableChooser<>();
    // m_driveChooser.addOption("ArcadeDrive1", drivingType.ArcadeDrive);
    // m_driveChooser.addOption("ArcadeDrive2", drivingType.ArcadeDrive2);
    // m_driveChooser.addOption("TankDrive", drivingType. TankDrive);
    // m_driveChooser.addOption("CurvatureDrive", drivingType.CurvatureDrive);
    // SmartDashboard.putData(m_driveChooser);
   
  }
  public void onAutoInit(){
    startTime = Timer.getFPGATimestamp();
    }
  
  public void onAutoPeriodic(){
 
  // Setpoint is implicitly 0, since we don't want the heading to change

  // Drives forward continuously at half speed, using the gyro to stabilize the heading


    while (delta_time < 2){
      delta_time = Timer.getFPGATimestamp()- startTime;
      driverTrain.voltDrive();
    
    }
    driverTrain.ArcadeDrive(0, 0);
    // To continue - add the shoot in the autonomous
  
  }
  public void onTeleopInit() {
     //new StartArcadeDrive(driverTrain).schedule();
    driverTrain.GyroToWidget();
    }

  public void onTeleopPeriodic(){

  }

  private void configureButtonBindings() {
    m_oi.button3.whileHeld(new CollectBalls(ballsCollector));
    m_oi.button4.whileHeld(new ReleaseBalls(ballsCollector));
    m_oi.button1.whileHeld(new ShootBall(ballsShooter));
    // m_oi.button5.whileHeld(new RollerWork(ballsRoller));
  }

//   public void getDrivingCommand(){
//     switch(m_driveChooser.getSelected()){
//       // case DO_NOTHING:
//       //  new WaitCommand(15);

//       case ArcadeDrive:
//       driverTrain.setDefaultCommand(new StartArcadeDrive(driverTrain));

//       case ArcadeDrive2:
//         driverTrain.setDefaultCommand(new StartArcadeDrive2(driverTrain));

//       case CurvatureDrive:
//       driverTrain.setDefaultCommand(new StartCurvatureDrive(driverTrain));

//       case TankDrive:
//       driverTrain.setDefaultCommand(new StartTankDrive(driverTrain));


//       default:
//         new WaitCommand(15);
//     }    
// }
}

