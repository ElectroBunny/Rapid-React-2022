// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  DoubleSolenoid DoublePCM = null;
  public boolean isIntakeOpen = false;
 
  public Climber() {
    DoublePCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    DoublePCM.set(Value.kOff);
  }

  public void offClimb() {
    DoublePCM.set(Value.kOff);
  }
// Opens the intake
  public void openClimb() {
    DoublePCM.set(Value.kForward);
  }

  public void closeClimb() {
    DoublePCM.set(Value.kReverse);
    isIntakeOpen = false;
  }



  }