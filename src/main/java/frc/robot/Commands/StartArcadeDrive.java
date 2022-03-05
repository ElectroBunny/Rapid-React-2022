// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.NewDriverTrain;
import frc.robot.OI;
import frc.robot.RobotMap;

public class StartArcadeDrive extends CommandBase {

  private static double kp = 0;
  private NewDriverTrain driver;
  double err = 0, kP = Preferences.getDouble("kPk", kp), kD = 0.02, errPast= 0;
  double turningValue = 0;
  boolean once = false;
  boolean drivewithfix = true;
  double currnetAngle = 0;
  double kp2=0;
  
  public StartArcadeDrive(NewDriverTrain innerDriver ) {
    driver = innerDriver;
    addRequirements(driver);
  
  }

  @Override
  public void initialize() {
    driver.changetoBrake();
  }

  @Override
  public void execute() {
    if(SmartDashboard.getBoolean("startDrive", false)){
    double yAxis = new OI().GetDriverRawAxis(RobotMap.STICK_Y);
    double xAxis = new OI().GetDriverRawAxis(RobotMap.STICK_X);

    if(xAxis > 0.35 || xAxis < -0.35){
     drivewithfix = false;
    } 
    else{
      drivewithfix = true;
    }

    if (drivewithfix){
      kP = Preferences.getDouble("kPk", kp);
      err = currnetAngle - driver.getGyro().getAngle();
      if(err>1.0)
        kp2=(0.45/err + 0.6/err); //check the fix with kp2
            
      
      turningValue = err * kP;
      if(Math.abs(turningValue) > 0.55){
      turningValue = 0.0;}

      driver.CurvatureDrive(-yAxis, turningValue);
    }

    else {
      driver.CurvatureDrive(-yAxis * 0.90, xAxis);
      currnetAngle = driver.getGyro().getAngle();
      driver.getGyro().reset();   
     }

   SmartDashboard.putNumber("err", err);
   SmartDashboard.putNumber("turningValue", turningValue);
   SmartDashboard.putNumber("xAxis", xAxis);
   SmartDashboard.putNumber("yAxis", yAxis);
    }
    else
    driver.getGyro().reset();   
  }

  @Override
  public void end(boolean interrupted) {
    driver.ArcadeDrive(0, 0);

  }

  @Override
  public boolean isFinished() {
    return false;
  }
  
  }



//   //driver.getGyro().calibrate(); //IF NOT WORK TRY THIS!
//   kP= SmartDashboard.getNumber("kP", 0.01);
//   double yAxis = new OI().GetDriverRawAxis(RobotMap.STICK_Y);
//   double xAxis = new OI().GetDriverRawAxis(RobotMap.STICK_X);
//   if(yAxis>0.09){
//   //   if(xAxis>0.2)
//   //   driver.ArcadeDrive(-yAxis, xAxis*0.3);
//   //   else{
//   err = 0.0 - driver.getGyro().getAngle();
//   turningValue = err * kP;
// if(!once){
//errPast=turningValue;
//once=true;
//}
//   driver.ArcadeDrive(-yAxis*0.7, turningValue+kD*(err-errPast));
//errPast=turningValue;
//   //}
// }

//  SmartDashboard.putNumberArray("Left output + Right output", driver.getMotorOutputPercent());
//  SmartDashboard.putNumber("err", err);
//  SmartDashboard.putNumber("turningValue", turningValue);
// }