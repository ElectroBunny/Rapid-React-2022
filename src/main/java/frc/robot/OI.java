// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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
    POVButton povbutton1= new POVButton(joystick_controller,180);
    POVButton povbutton2= new POVButton(joystick_controller,0);


    XboxController xbox_controller = new XboxController(RobotMap.XBOX_CONTROLLER);
     public Button L1 = new JoystickButton(xbox_controller, 11);
     public Button R1 = new JoystickButton(xbox_controller, 12);

    /*
      private final XboxController m_joystick = new XboxController(0);
    final JoystickButton l2 = new JoystickButton(m_joystick, 9);
    final JoystickButton r2 = new JoystickButton(m_joystick, 10);
    final JoystickButton r1 = new JoystickButton(m_joystick, 12);
    */

    public double GetDriverRawAxis(int axis) {
        return joystick_controller.getRawAxis(axis);
    }

    public Joystick getJoystick() {
        return joystick_controller;
    }
    
    public XboxController getXbox360Joystick() {
        return xbox_controller;
    }
    

    public double GetXboxLEFTYRawAxis() {
        return xbox_controller.getLeftY();
    }
    public double GetXboxRIGHTXRawAxis( ) {
        return xbox_controller.getRightX();
    }



}

// xboxPad.getX(GenericHID.Hand.kLeft);