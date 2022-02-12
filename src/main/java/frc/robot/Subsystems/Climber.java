// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.Subsystems;
// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
// import frc.robot.RobotMap;


// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Climber extends SubsystemBase {
//   DoubleSolenoid pitchSolenoid = null;
//   public Climber() {
//     pitchSolenoid = new DoubleSolenoid(null, RobotMap.SHOOTER_PITCH_SOLENOID_DEPLOY, RobotMap.SHOOTER_PITCH_SOLENOID_RETRACT);
//   }
//   public void pitchUp(){
//    pitchSolenoid.set(Value.kForward);
//   }

//   public void pitchDown(){
//     pitchSolenoid.set(Value.kReverse);
//   }
//   public void pitchOff(){
//     pitchSolenoid.set(Value.kOff);
//   }
  
//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
//   public void initDefaultCommand() {
//     // Set the default command for a subsystem here.
//     // setDefaultCommand(new MySpecialCommand());
//     }
// }
