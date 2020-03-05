/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
 
  //CAN ports for motors
  public static int leftMasterPort = 1;
  public static int leftSlavePort = 3;
  public static int rightMasterPort = 2;
  public static int rightSlavePort = 4;
  public static int topShooterMotorPort = 5;
  public static int bottomShooterMotorPort = 6;
  public static int liftMasterPort = 7;
  public static int liftSlavePort = 8;
  public static int magazineMotorPort = 9;
  public static int intakeRollerMotorPort = 10;
  public static int transferConveyorMotorPort = 11;
  public static int colorSpinnerMotorPort = 12;
 
  //joysticks
  public static int joyStickPort = 0;
  public static int joyStickPort2 = 1;
    
  //Joystick Buttons
  public static int A = 1;
  public static int B = 2;
  public static int X = 3;
  public static int Y = 4;
  public static int LBumpr = 5;
  public static int RBumpr = 6;
  public static int LStick = 9;
  public static int RStick = 10;
  public static int START = 8;
  public static int BACK = 7;

  //Digital inputs
  public static int transferBeamBreakPort        = 1;
  public static int magBeamBreakPort             = 0;
  

  
  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;

  //constants for motor acceleration and max speed. Used in drive subsystem
  public static double maxAccel = .02;
  public static double maxSpeed = 1.0;
  public static double turnMultiplier = .7;

  //constants for aiming
  public static double fastAimSpeed = 0.6;
  public static double slowAimSpeed = 0.35;
 
  //deadzone for when the Aim method stops turning the robot.
  public static double aimDeadZone = 1.0;


  //default shooter speed. .4 seems good at 10 feet
  public static double ShooterDefaultSpeed = .50;
 
  //percentage that top motor will slow to for backspin. (set to 1 to make them spin equally)
  public static double topShooterPercentage = .75;
 
  //for shooter upToSpeed boolean
  public static double upToSpeedRange = 400;
  public static double shooterTimout = 50;
  

  //lift heights
  public static double liftHeightMax = 220; //actual max 500-ish. leaving at 220ish to get to bar height.
  public static double liftHeightHome = 10.0;//set to 10 for safety.
  public static double liftHeightWheelSetup = 100.0;
  public static double liftHeightWheelSpin = 90.0;


  //constants for Spark Max motor controller types. cleans up some of the drive subsystem code IMO.
  public static CANSparkMaxLowLevel.MotorType NEO = CANSparkMaxLowLevel.MotorType.kBrushless;
  public static CANSparkMaxLowLevel.MotorType neo550 = CANSparkMaxLowLevel.MotorType.kBrushless;

  public static double positionRangeModifier = .15;
  public static double magJogSpeed = 0.1;

}
