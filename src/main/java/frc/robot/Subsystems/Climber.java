// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
  DoubleSolenoid DoublePCMRight = null;
  DoubleSolenoid DoublePCMLeft = null;
  public boolean isIntakeOpen = false;
 
  public Climber() {
    DoublePCMRight = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.RIGHT_SOLENOID_FW, RobotMap.RIGHT_SOLENOID_BW);
    DoublePCMLeft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.LEFT_SOLENOID_FW, RobotMap.LEFT_SOLENOID_BW);
    DoublePCMRight.set(Value.kOff);
    DoublePCMLeft.set(Value.kOff);
  }

  public void offClimb() {
    DoublePCMRight.set(Value.kOff);
    DoublePCMLeft.set(Value.kOff);
  }
// Opens the intake
  public void openClimb() {
    DoublePCMRight.set(Value.kForward);
    DoublePCMLeft.set(Value.kForward);
  }

  public void closeClimb() {
    DoublePCMRight.set(Value.kReverse);
    DoublePCMLeft.set(Value.kReverse);
    isIntakeOpen = false;
  }
  }