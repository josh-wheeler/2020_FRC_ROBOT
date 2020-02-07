/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ShooterSpinCommand;
import frc.robot.commands.ShooterStopCommand;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);  
  // Button button = new JoystickButton(stick, buttonNumber);
  public XboxController joystick = new XboxController(RobotMap.joyStickPort);
  
  public Button A = new JoystickButton(joystick, RobotMap.A);
  public Button B = new JoystickButton(joystick, RobotMap.B);
  public Button X = new JoystickButton(joystick, RobotMap.X);
  public Button Y = new JoystickButton(joystick, RobotMap.Y);
  public Button BACK = new JoystickButton(joystick, RobotMap.BACK);
  public Button START = new JoystickButton(joystick, RobotMap.START);
  public Button LStick = new JoystickButton(joystick, RobotMap.LStick);
  public Button RStick = new JoystickButton(joystick, RobotMap.RStick);
  public Button LBumper = new JoystickButton(joystick, RobotMap.LBumpr);
  public Button RBumper = new JoystickButton(joystick, RobotMap.RBumpr);

  public OI(){
    //old method for buttons - UPDATE: only method for buttons, it is WPILIB specific, and would require a lot of additions to match WPILIB's functionality.
    Y.whenPressed(new ShooterSpinCommand(-RobotMap.highShotSpeed, RobotMap.highShotSpeed));
    B.whenPressed(new ShooterSpinCommand(-RobotMap.lowShotSpeed, RobotMap.lowShotSpeed));
    X.whenPressed(new ShooterStopCommand());
    A.whenPressed(new ShooterSpinCommand(-RobotMap.topVarShotSpeed, RobotMap.bottomVarShotSpeed));



    //joystick trigger stuff. This should be what we'd need for variable speed control on a motor using the triggers as pressure sensitive input devices.
    //joystick.getTriggerAxis(Hand.kRight);


    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.

    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:

    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());

    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());

    // Start the command when the button is released and let it run the command
    // until it is finished as determined by it's isFinished method.
    //RBumper.whenReleased(new TestButtonCommand());
  }
}
