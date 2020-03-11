/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.AimCommand;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.CHOOTCommand;
import frc.robot.commands.EmptyShooterCommand;
import frc.robot.commands.IntakeActiveToggle;
import frc.robot.commands.JogLiftCommand;
import frc.robot.commands.LiftMoveToPositionCommand;
import frc.robot.commands.LiftStopCommand;
import frc.robot.commands.ShooterSpinCommand;
import frc.robot.commands.ShooterStopCommand;
import frc.robot.commands.clearBallsPresent;
import frc.robot.subsystems.ScissorLiftSubsystem.liftPosition;

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
  public XboxController pilot = new XboxController(RobotMap.joyStickPort);
  public XboxController copilot = new XboxController(RobotMap.joyStickPort2);

  
  public Button A1 = new JoystickButton(pilot, RobotMap.A);
  public Button B1 = new JoystickButton(pilot, RobotMap.B);
  public Button X1 = new JoystickButton(pilot, RobotMap.X);
  public Button Y1 = new JoystickButton(pilot, RobotMap.Y);
  public Button BACK1 = new JoystickButton(pilot, RobotMap.BACK);
  public Button START1 = new JoystickButton(pilot, RobotMap.START);
  public Button LStick1 = new JoystickButton(pilot, RobotMap.LStick);
  public Button RStick1 = new JoystickButton(pilot, RobotMap.RStick);
  public Button LBumper1 = new JoystickButton(pilot, RobotMap.LBumpr);
  public Button RBumper1 = new JoystickButton(pilot, RobotMap.RBumpr);

  public Button A2 = new JoystickButton(copilot, RobotMap.A);
  public Button B2 = new JoystickButton(copilot, RobotMap.B);
  public Button X2 = new JoystickButton(copilot, RobotMap.X);
  public Button Y2 = new JoystickButton(copilot, RobotMap.Y);
  public Button BACK2 = new JoystickButton(copilot, RobotMap.BACK);
  public Button START2 = new JoystickButton(copilot, RobotMap.START);
  public Button LStick2 = new JoystickButton(copilot, RobotMap.LStick);
  public Button RStick2 = new JoystickButton(copilot, RobotMap.RStick);
  public Button LBumper2 = new JoystickButton(copilot, RobotMap.LBumpr);
  public Button RBumper2 = new JoystickButton(copilot, RobotMap.RBumpr);



  public OI(){

    //PILOT CONTROLS
    Y1.whenPressed(new IntakeActiveToggle());
    //START1.whenPressed(new ShooterSpinCommand());
    //BACK1.whenPressed(new clearBallsPresent());
    
    //COPILOT CONTROLS
    //A2.whenPressed(new LiftMoveToPositionCommand(liftPosition.home));
    Y2.whenPressed(new EmptyShooterCommand(20));
    START1.whenPressed(new ShooterSpinCommand(.65));
    BACK1.whenPressed(new clearBallsPresent());
    A1.whileHeld(new AimCommand());
    B1.whenPressed(new CHOOTCommand());
    X1.whenPressed(new ShooterStopCommand());


    //X2.whenPressed(new LiftMoveToPositionCommand(liftPosition.wheelSetup));
    //Y2.whenPressed(new LiftMoveToPositionCommand(liftPosition.maxHeight));
    RBumper2.whileHeld(new JogLiftCommand(1));
    LBumper2.whileHeld(new JogLiftCommand(-1)); 

    //LBumper1.whenPressed(new ShooterSpinCommand(.5)); 

    

    //LBumper.whenPressed(new LimelightTargetCamCommand());




    //joystick trigger stuff. This should be what we'd need for variable speed control on a motor using the triggers as pressure sensitive input devices.


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
  public double rightTriggerAxis(){
    return pilot.getTriggerAxis(Hand.kRight);
  }
  public double leftTriggerAxis(){
    return pilot.getTriggerAxis(Hand.kLeft);
  }

}
