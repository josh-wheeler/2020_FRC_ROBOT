/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.LiftStopCommand;

/**
 * Add your docs here.
 */
public class ScissorLiftSubsystem extends Subsystem {

  CANSparkMax liftMasterMotor = new CANSparkMax(RobotMap.liftMasterPort, RobotMap.NEO);
  CANSparkMax liftSlaveMotor = new CANSparkMax(RobotMap.liftSlavePort, RobotMap.NEO);
  CANEncoder masterEncoder = liftMasterMotor.getEncoder();
  CANEncoder slaveEncoder = liftSlaveMotor.getEncoder();
  CANPIDController masterPID = liftMasterMotor.getPIDController();
  CANPIDController slavePID = liftSlaveMotor.getPIDController();
  public DigitalInput downSwitch = new DigitalInput(RobotMap.leftScissorLimitSwitchPort);

  private double kP = 0.00003; // .5
  private double kI = 2e-6; // .0
  private double kD = 1e-6;//0.000001;
  private double kIz = 5;
  private double kFF = 0.0;
  private double kMaxOutput = 1; 
  private double kMinOutput = -1;

  //smartmotion coefficients. set these to 
  //private double maxVel = .5; //RPM the lift will move at full speed
  //private double maxAccel = .1;//RPMs per second it can increase until it hits maxVel

  private double setPosition = 0.0;
  public boolean homedOut;

  public enum liftPosition{home, wheelSetup, wheelEngage, maxHeight}

  //constructor
  public ScissorLiftSubsystem(){

    liftMasterMotor.restoreFactoryDefaults();
    liftSlaveMotor.restoreFactoryDefaults();                   
    liftSlaveMotor.follow(liftMasterMotor);

    liftMasterMotor.setIdleMode(IdleMode.kBrake);
    
    masterPID.setP(kP);
    masterPID.setI(kI);
    masterPID.setD(kD);
    masterPID.setIZone(kIz);
    masterPID.setFF(kFF);
    masterPID.setOutputRange(kMinOutput, kMaxOutput);

    //masterPID.setSmartMotionMaxVelocity(maxVel, slotID);
    //masterPID.setSmartMotionMaxAccel(maxAccel, slotID);

   
    //if(downSwitch.get()){setHomePosition();}else{homedOut=false;}

    setHomePosition();
    //liftPIDStatus();

  }
  public void liftPIDStatus(){
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    //display smartmotion coefficients
    SmartDashboard.putNumber("Motor Output", liftMasterMotor.getAppliedOutput());
    //SmartDashboard.putNumber("Max Acceleration", maxAccel);
    //SmartDashboard.putNumber("Max Velocity", maxVel);

  }
  //updates smartdashboard. called in robot's periodic method
  public void liftStatus(){
    SmartDashboard.putNumber("Lift Motor Speed", masterEncoder.getVelocity());
    SmartDashboard.putNumber("Lift Motor Position", masterEncoder.getPosition());
    SmartDashboard.putNumber("Set Position Variable", setPosition);
    SmartDashboard.putBoolean("Lift Homed Out", homedOut);

  }
  
  public void zeroSetPosition(){
    setPosition = 0.0; 
  }
  
  public void moveToPosition(liftPosition setting){

    setPosition = liftNumber(setting);
    System.out.println("lift setting" + setting);

    if(homedOut){
     //masterPID.setReference(setPosition, ControlType.kSmartMotion);
     masterPID.setReference(setPosition, ControlType.kPosition);
     System.out.println("set position" + setPosition);

    }
  }
  //constants fo lift jogging
  private int liftSafeUp = 350;
  private int liftSafeDown = 10;
  private double liftSafeJog = .1;
  private double liftJog = .5;

  public void jogLift(double amount){

    if(masterEncoder.getPosition() < liftSafeDown || masterEncoder.getPosition() > liftSafeUp)
    amount = Math.signum(amount) * liftSafeJog;
    else
    amount = Math.signum(amount) * liftJog;


    liftMasterMotor.set(amount);
  }
  
  public void stopLift(){   
    liftMasterMotor.set(0.0);
  }


  private void setHomePosition(){
    masterEncoder.setPosition(0.0);
    homedOut = true;
  }
  
  private double liftNumber(liftPosition position){
    double output;
    switch(position){
      case home:
      output = RobotMap.liftHeightHome;
      break;
      case maxHeight:
      output = RobotMap.liftHeightMax;
      break;
      case wheelEngage:
      output = RobotMap.liftHeightWheelSpin;
      break;
      case wheelSetup:
      output = RobotMap.liftHeightWheelSetup;
      break;
      default:
      output = setPosition;
      break;
    }
    return output;
  }
  public void LiftMotorTuner(){
   // read PID coefficients from SmartDashboard
   double p = SmartDashboard.getNumber("P Gain", 0);
   double i = SmartDashboard.getNumber("I Gain", 0);
   double d = SmartDashboard.getNumber("D Gain", 0);
   double iz = SmartDashboard.getNumber("I Zone", 0);
   double ff = SmartDashboard.getNumber("Feed Forward", 0);
   double max = SmartDashboard.getNumber("Max Output", 0);
   double min = SmartDashboard.getNumber("Min Output", 0);
   //double maxV = SmartDashboard.getNumber("Max Velocity", 0);
   //double minV = SmartDashboard.getNumber("Min Velocity", 0);
   //double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
   //double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);

   // if PID coefficients on SmartDashboard have changed, write new values to controller
   if((p != kP)) { masterPID.setP(p); kP = p; }
   if((i != kI)) { masterPID.setI(i); kI = i; }
   if((d != kD)) { masterPID.setD(d); kD = d; }
   if((iz != kIz)) { masterPID.setIZone(iz); kIz = iz; }
   if((ff != kFF)) { masterPID.setFF(ff); kFF = ff; }
   if((max != kMaxOutput) || (min != kMinOutput)) { 
      masterPID.setOutputRange(min, max); 
     kMinOutput = min; kMaxOutput = max; 
   }
   //if((maxV != maxVel)) { masterPID.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
   //if((minV != minVel)) { masterPID.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
   //if((maxA != maxAccel)) { masterPID.setSmartMotionMaxAccel(maxA,0); maxAccel = maxA; }
  // if((allE != allowedErr)) { masterPID.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
   setDefaultCommand(new LiftStopCommand());
  }
}
