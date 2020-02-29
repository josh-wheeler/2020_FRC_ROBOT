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
 

  //constants for motor acceleration and max speed. Used in drive subsystem
  public static double maxAccel = .015;
  public static double maxSpeed = 1.0;
  public static double turnMultiplier = .5;

  //constants for aiming
  public static double fastAimSpeed = 0.5;
  public static double slowAimSpeed = 0.3;
 
  //deadzone for when the Aim method stops turning the robot.
  public static double aimDeadZone = 1.0;


  //default shooter speed. .4 seems good at 10 feet
  public static double ShooterDefaultSpeed = .40;
  //percentage that top motor will slow to for backspin. (set to 1 to make them spin equally)
  public static double topShooterPercentage = .75;
  //for shooter upToSpeed boolean
  public static double upToSpeedRange = 300.0;
  public static double shooterTimout = 50;
 
  //other constants
  public static int wheelDiam = 8;
  //lift heights
  public static double liftHeightMax = 500; //actual max 500-ish. leaving low for testing purposes
  public static double liftHeightHome = 5.0;//set to 5 so we aren't constantly contacting the limit switch.
  public static double liftHeightWheelSetup = 100.0;
  public static double liftHeightWheelSpin = 90.0;
  public static double liftJogSpeed = 0.5; //0.4 for competition

  //constants for AIM method
    //math for dist:  d = (heightoftarget-heightofcamera) / tan(angleofcamera + angletotarget)
    //length of field: roughly 578. 
    //dividing by this gives us a percentage for the motors (if we are 578 inches from target, output =1 full power)
    public static double distanceCoefficient = 578;
    
    //limelight angle: 45 target height: 98.25 in (center of inside upper target) Limelight height: 22.25 in
    public static double limelightHeight = 22.25;
    public static double targetHeight = 98.25;
    public static double limelightAngle = 45; 

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

  //Digital inputs
  public static int leftScissorLimitSwitchPort   = 3;
  public static int rightScissorLimitSwitchPort  = 4;
  public static int transferBeamBreakPort        = 1;
  public static int magBeamBreakPort             = 2;
  public static int absEncoderPort               = 0;

  
  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;


  //constants for Spark Max motor controller types. cleans up some of the drive subsystem code IMO.
  public static CANSparkMaxLowLevel.MotorType NEO = CANSparkMaxLowLevel.MotorType.kBrushless;
  public static CANSparkMaxLowLevel.MotorType neo550 = CANSparkMaxLowLevel.MotorType.kBrushless;

  public static double positionRangeModifier = .15;
  public static double magPos_1 = 0.0;
  public static double magPos_2 = 0.33;
  public static double magPos_3 = 0.66;
  public static double magJogSpeed = 0.1; //0.4 for competition
  public static double intakeJogSpeed = 0.4;

}
