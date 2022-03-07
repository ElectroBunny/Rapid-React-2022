// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class OI {

    Joystick joystick_controller = new Joystick(RobotMap.JOYSTICK_CONTROLLER);
    Button button1 = new JoystickButton(joystick_controller, 1);
    Button button2 = new JoystickButton (joystick_controller, 2);
    Button button3 = new JoystickButton(joystick_controller, 3);
    Button button4 = new JoystickButton(joystick_controller, 4);
    Button button5 = new JoystickButton(joystick_controller, 5);
    Button button6 = new JoystickButton(joystick_controller, 6);
    Button button7 = new JoystickButton(joystick_controller, 7);
    Button button8 = new JoystickButton(joystick_controller, 8);
    Button button9 = new JoystickButton(joystick_controller, 9);
    Button button10 = new JoystickButton(joystick_controller, 10);
    POVButton povbutton1= new POVButton(joystick_controller,180);
    POVButton povbutton2= new POVButton(joystick_controller,0);
//-------------------------------------------------------------------------------//
    XboxController xbox_controller = new XboxController(RobotMap.XBOX_CONTROLLER);
    public Button xbox1 = new JoystickButton(xbox_controller, 1); 
    public Button xbox2 = new JoystickButton(xbox_controller, 2);
    public Button xbox3 = new JoystickButton(xbox_controller, 3);
    public Button xbox4 = new JoystickButton(xbox_controller, 4);
    public Button xbox5 = new JoystickButton(xbox_controller, 5); 
    public Button xbox6 = new JoystickButton(xbox_controller, 6);
    public Button xbox7 = new JoystickButton(xbox_controller, 7);
    public Button xbox8 = new JoystickButton(xbox_controller, 8);
    public Button xbox9 = new JoystickButton(xbox_controller, 9); 
    public Button xbox10 = new JoystickButton(xbox_controller, 10);
    public Button xbox11 = new JoystickButton(xbox_controller, 11);
    public Button xbox12 = new JoystickButton(xbox_controller, 12);
//---------------------------------------------------------------------------------//   
    PS4Controller ps4_controller = new PS4Controller(RobotMap.PS4_CONTROLLER);
    
    public Button b1 = new JoystickButton(ps4_controller, 10);
        //ADD BUTTON//


    public double GetDriverRawAxis(int axis) {
        return joystick_controller.getRawAxis(axis);
    }

    public Joystick getJoystick() {
        return joystick_controller;
    }
    
    public XboxController getXbox360Joystick() {
        return xbox_controller;
    }

    public PS4Controller getPS4Joystick() {
        return ps4_controller;
    }
    

    public double GetXboxLEFTYRawAxis() {
        return xbox_controller.getLeftY();
    }
    public double GetXboxRIGHTXRawAxis( ) {
        return xbox_controller.getRightX();
    }
    public void buttonsXbox(){
        SmartDashboard.putBoolean("xbox1",xbox1.get());
        SmartDashboard.putBoolean("xbox2",xbox2.get());
        SmartDashboard.putBoolean("xbox3",xbox3.get());
        SmartDashboard.putBoolean("xbox4",xbox4.get());
        SmartDashboard.putBoolean("xbox5",xbox5.get());
        SmartDashboard.putBoolean("xbox6",xbox6.get());
        SmartDashboard.putBoolean("xbox7",xbox7.get());
        SmartDashboard.putBoolean("xbox8",xbox8.get());
        SmartDashboard.putBoolean("xbox9",xbox9.get());
        SmartDashboard.putBoolean("xbox10",xbox10.get());
        SmartDashboard.putBoolean("xbox11",xbox11.get());
        SmartDashboard.putBoolean("xbox12",xbox12.get());

    }



}

// xboxPad.getX(GenericHID.Hand.kLeft);