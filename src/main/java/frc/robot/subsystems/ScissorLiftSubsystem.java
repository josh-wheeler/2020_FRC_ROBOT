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
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

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
  //CANSparkMax liftSlaveMotor = new CANSparkMax(RobotMap.liftSlavePort, RobotMap.NEO);
  CANEncoder masterEncoder = liftMasterMotor.getEncoder();
 // CANEncoder rightEncoder = rightLiftMotor.getEncoder();
  CANPIDController masterPID = liftMasterMotor.getPIDController();
  //CANPIDController rightPID = rightLiftMotor.getPIDController();
  
  private double kP = 0.03; // .5
  private double kI = 2e-6; // .0
  private double kD = 1e-6;//0.000001;
  private double kIz = 2;
  private double kFF = 0.0;
  private double kMaxOutput = 1; 
  private double kMinOutput = -1;
  private double setPosition = 0.0;
  public boolean homedOut;
  public DigitalInput downSwitch = new DigitalInput(RobotMap.leftLowerLiftLimitSwitchPort);
  

  public enum liftPosition{home, wheelSetup, wheelEngage, maxHeight}

//constructor
  public ScissorLiftSubsystem(){

    liftMasterMotor.restoreFactoryDefaults();

    liftMasterMotor.setIdleMode(IdleMode.kBrake);
    //liftMasterMotor.setClosedLoopRampRate(2.0);
    
    masterPID.setP(kP);
    masterPID.setI(kI);
    masterPID.setD(kD);
    masterPID.setIZone(kIz);
    masterPID.setFF(kFF);
    masterPID.setOutputRange(kMinOutput, kMaxOutput);
   
    if(downSwitch.get()){
      
      masterEncoder.setPosition(0.0);
      homedOut = true;
      
    }

    //liftSlaveMotor.follow(liftMasterMotor);
 
    
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    //SmartDashboard.putNumber("Set Rotations", 0);
    
  }

  //updates smartdashboard. called in robot's periodic method
  public void liftStatus(){
    SmartDashboard.putNumber("Lift Motor Speed", masterEncoder.getVelocity());
    SmartDashboard.putNumber("Lift Motor Position", masterEncoder.getPosition());
    SmartDashboard.putNumber("Set Position Variable", setPosition);
    //SmartDashboard.putBoolean("Target Reached boolean", targetReached);

  }
  
  public void zeroSetPosition(){
    setPosition = 0.0;
  }
  
  public void moveToPosition(liftPosition setting){

    double position = liftNumber(setting);
    masterPID.setReference(position, ControlType.kPosition);
  }

  public void jogLift(double amount){
    amount = Math.signum(amount) * .1;
    liftMasterMotor.set(amount);
  }
  
  public void stopLift(){   
    liftMasterMotor.set(0.0);
  }
  public void homeOutLift(){
    if(!homedOut){
      while(!downSwitch.get()){
        liftMasterMotor.set(-.01);
      }
    }
    if(downSwitch.get()){
      homedOut = true;
      zeroLiftEncoder();
      //moveToPosition(liftPosition.home);
      //zeroLiftEncoder();

    }
  }

  public void zeroLiftEncoder(){
    masterEncoder.setPosition(0.0);
  }
  private double liftNumber(liftPosition position){
    double output;
    switch(position){
      case home:
      output = RobotMap.liftHeightHome;
      case maxHeight:
      output = RobotMap.liftHeightMax;
      case wheelEngage:
      output = RobotMap.liftHeightWheelSpin;
      case wheelSetup:
      output = RobotMap.liftHeightWheelSetup;
      default:
      output = setPosition;
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

  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
   setDefaultCommand(new LiftStopCommand());
  }
}
