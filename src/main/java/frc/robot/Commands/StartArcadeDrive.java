// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.NewDriverTrain;
import frc.robot.OI;

public class StartArcadeDrive extends CommandBase {

  private NewDriverTrain driver;
  // double err = 0, kP = Preferences.getDouble("kPk", kp), kD = 0.02, errPast= 0;
  // double turningValue = 0;
  // boolean once = false;
  // boolean drivewithfix = true;
  // double currnetAngle = 0;
  // double kp2=0;
  // double gainY=0.65, gainX=0.35;
  // double startTime, deltaTime;
  // double inclick=0.2;

  double forwardY = 0, reverseY = 0, xAxis = 0;


   public StartArcadeDrive(NewDriverTrain innerDriver ) {
    driver = innerDriver;
    addRequirements(driver);
  
  }
  
  @Override
  public void initialize() {

  }

  @Override
  public void execute() {

     forwardY = new OI().getXbox360Joystick().getRightTriggerAxis();
     reverseY = new OI().getXbox360Joystick().getLeftTriggerAxis();
     xAxis = new OI().getXbox360Joystick().getLeftX();

    //  SmartDashboard.putNumber("POV 0",(new OI().getXbox360Joystick().getPOV(0)));


    // if(deltaTime > inclick){
    //  if(new OI().getXbox360Joystick().getPOV(0)==0)
    //   gainY += 0.05;
    //  if(new OI().getXbox360Joystick().getPOV(0)==90)
    //   gainX += 0.05;
    //  if(new OI().getXbox360Joystick().getPOV(0)==180)
    //   gainY -= 0.05;
    //  if(new OI().getXbox360Joystick().getPOV(0)==270)
    //  gainX -= 0.05;
     
    //  inclick += 0.2;
    // }
    // if(gainY>1)
    // gainY = 1;

    // if(gainY<0)
    // gainY = 0;
    
    // if(gainX>1)
    // gainX = 1;

    // if(gainX<0)
    // gainX=0;

    //  SmartDashboard.putNumber("forwardY", forwardY);
    //  SmartDashboard.putNumber("reverseY", reverseY);
    //  SmartDashboard.putNumber("xAxis", xAxis);
    //  SmartDashboard.putNumber("Gain Y", gainY);
    //  SmartDashboard.putNumber("Gain X", gainX);
    //  SmartDashboard.putNumber("DelaTime ", deltaTime);


     driver.ArcadeDrive((forwardY - reverseY)*0.95, xAxis * 0.85);

  }
    
  @Override
  public void end(boolean interrupted) {
    driver.ArcadeDrive(0, 0);
    driver.changetoCoast();

  }

  @Override
  public boolean isFinished() {
    return false;
  }
  
  }


//   private NewDriverTrain driver;
//   double err = 0, kP = 0, kD = 0.02, errPast= 0;
//   double turningValue = 0;
//   boolean once = false;
//   boolean drivewithfix = true;
//   double currnetAngle = 0;
//   double kp2=0;
  
//   public StartArcadeDrive(NewDriverTrain innerDriver ) {
//     driver = innerDriver;
//     addRequirements(driver);
  
//   }

//   @Override
//   public void initialize() {
//   }

//   @Override
//   public void execute() {
//    if(SmartDashboard.getBoolean("startDrive", false)){
//     double yAxis = new OI().GetDriverRawAxis(RobotMap.STICK_Y);
//     double xAxis = new OI().GetDriverRawAxis(RobotMap.STICK_X);
//     driver.ArcadeDrive(-yAxis, xAxis);
  
//     if(Math.abs(xAxis)>0.25){
//      drivewithfix = false;
//     } 
//     else{
//       drivewithfix = true;
//     }

//     if (drivewithfix){
//       err = currnetAngle - Math.IEEEremainder(driver.getGyro().getAngle(), 360);
//       kP=FixkP(err);
//       turningValue = err * kP;

//       if(Math.abs(turningValue) > 0.65)
//         turningValue = 0.0;

//       driver.ArcadeDrive(-yAxis, turningValue);
//     }
//       else {
//       driver.ArcadeDrive(-yAxis * 0.90, xAxis);
//       currnetAngle = Math.IEEEremainder(driver.getGyro().getAngle(), 360);
//      }

//    SmartDashboard.putNumber("err", err);
//    SmartDashboard.putNumber("turningValue", turningValue);
//    SmartDashboard.putNumber("xAxis", xAxis);
//    SmartDashboard.putNumber("yAxis", yAxis);
//     }
  
//     else
//     driver.getGyro().reset();   
//   }


//   @Override
//   public void end(boolean interrupted) {
//     driver.ArcadeDrive(0, 0);

//   }

//   @Override
//   public boolean isFinished() {
//     return false;
//   }
  
  
  

//   public double FixkP(double err){
//   double kP_fix=0.0;
  
//     if (Math.abs(err)<1)
//         {
//            kP_fix=0;
//         }
//        else
//        {
//             if (Math.abs(err)<3)
//           {
//             kP_fix=0.135;
//           }
//           else
//           {
//             if(Math.abs(err)<5)
//             kP_fix=0.085;
//             else
//             {
//               if(Math.abs(err)<10){
//                 kP_fix=0.045;
//             }
//             else
//             {
//               if(Math.abs(err)<15){
//                 kP_fix=0.03;
//               }
//               else
//               {
//                 kP_fix=0.0;
//               }
//             }
//           }
//         }
//       }
//       return kP_fix;
//   }
// }




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