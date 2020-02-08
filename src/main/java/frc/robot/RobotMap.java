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

  //constants for motor acceleration and max speed. Used in drive subsystem
  public static double maxAccel = .02;
  public static double maxSpeed = 1.0;
  public static double turnMultiplier = .5;
  //constants for shooter low and high speed shots
  public static double lowShotSpeed = .1;
  public static double highShotSpeed = .8;
  //for variable speed shots
  public static double topShooterMotorSpeed = .30;
  public static double bottomShooterMotorSpeed = .40;
  //for shooter upToSpeed boolean
  public static double upToSpeedRange = 150.0;
  //other constants
  public static int wheelDiam = 6;

  public static double aimIncrement = 0.1;
  public static double aimDeadzone = 0.08;

  //joysticks
  public static int joyStickPort = 0;
    
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

  
  
  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;


  //constants for Spark Max motor controller types. cleans up some of the drive subsystem code IMO.
  public static CANSparkMaxLowLevel.MotorType NEO = CANSparkMaxLowLevel.MotorType.kBrushless;
  public static CANSparkMaxLowLevel.MotorType neo550 = CANSparkMaxLowLevel.MotorType.kBrushless;

}
