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

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class BallMagazineSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  CANSparkMax magazineMotor = new CANSparkMax(RobotMap.magazineMotorPort, RobotMap.neo550);
  CANEncoder magazineEncoder = magazineMotor.getEncoder();
  CANPIDController magazinePID = magazineMotor.getPIDController();
  DutyCycleEncoder revAbsoluteEncoder = new DutyCycleEncoder(RobotMap.absEncoderPort);

  public BallMagSlot BS1 = new BallMagSlot();
  public BallMagSlot BS2 = new BallMagSlot();
  public BallMagSlot BS3 = new BallMagSlot();

  //number of rotations required to advance the ball 120°. 
  private double rotateAmount = 10;
  //number to add rotateAmount to so motor always travels in one direction.
  private double currentSetPosition = 0;

  private double kP = 0.00003; // .5
  private double kI = 2e-6; // .0
  private double kD = 1e-6;//0.000001;
  private double kIz = 5;
  private double kFF = 0.0;
  private double kMaxOutput = 1; 
  private double kMinOutput = -1;



//-----------------------------------------

  public BallMagazineSubsystem(){

    magazineMotor.restoreFactoryDefaults();

    magazineMotor.setIdleMode(IdleMode.kBrake);
    magazinePID.setP(kP);
    magazinePID.setI(kI);
    magazinePID.setD(kD);
    magazinePID.setIZone(kIz);
    magazinePID.setFF(kFF);
    magazinePID.setOutputRange(kMinOutput, kMaxOutput);

    magazineEncoder.setPosition(0.0);

    setCurrentMagPos();

    //the next lines set the ball present for competition pre-loading. this must be done after setCurrentMagPos() since that sets
    //ballpresent at load position to false
    BS1.ballPresent=true;
    BS2.ballPresent=true;
    BS3.ballPresent=true;

  }

  public void revolve(){
        
      //this adds 120° of movement each time, so the motor only goes one direction.
      //I'm using the motor's PID and built in encoder to move to position, but the absolute rev encoder to set where the shaft is. this is for slippage, and may have been unneccesary
      currentSetPosition = currentSetPosition + rotateAmount;
      magazinePID.setReference(currentSetPosition, ControlType.kPosition);
      setCurrentMagPos();
    
  }

 public boolean readyToLoad(){
  //this checks to see if the magazine slot at load position is ready to load(pretty self explanatory)
    if(BS1.atLoadPos && !BS1.ballPresent)
      return true;
    else if(BS2.atLoadPos && !BS2.ballPresent)
      return true;
    else if(BS3.atLoadPos && !BS3.ballPresent)
      return true;
    else
     return false;
  }

  public boolean oneInTheChamber(){
    //this figures out which position is at load position and then checks to see if there is a ball at the corresponding shooter position.
    if(BS1.atLoadPos && BS2.ballPresent)
      return true;
    else if(BS2.atLoadPos && BS3.ballPresent)
      return true;
    else if(BS3.atLoadPos && BS2.ballPresent)
      return true;
    else
    return false;
  }

  public void loadBall(){
    
    if(BS1.atLoadPos)
      BS1.setBallLoaded();
    if(BS2.atLoadPos)
      BS2.setBallLoaded();
    if(BS3.atLoadPos)
      BS3.setBallLoaded();
      

    if(!oneInTheChamber())
      revolve();
  }

  public void magazineStatus(){
    SmartDashboard.putNumber("Abs Encoder GET()",revAbsoluteEncoder.get());
    SmartDashboard.putNumber("getAbsEncoderPos result",getAbsEncoderPos());

  }

  public void CHOOT(){
    if(Robot.shooterSubsystem.upToSpeed()){
      if(BS1.ballPresent || BS2.ballPresent || BS3.ballPresent){
        revolve();
      }
    }
  }



  public void setCurrentMagPos(){
   double curPos = getAbsEncoderPos();

   if(positionRange(curPos, RobotMap.magPos_1)){
     BS1.setAtLoadPosition();
   }
   if(positionRange(curPos, RobotMap.magPos_2)){
     BS2.setAtLoadPosition();
   }
   if(positionRange(curPos, RobotMap.magPos_3)){
     BS3.setAtLoadPosition();
   }
  }
  public double getAbsEncoderPos(){
    double raw = revAbsoluteEncoder.get();
    raw = raw - (int)raw;
    return Math.abs(raw);

  }
  public void jogMag(double amount){
    amount = Math.signum(amount) * RobotMap.magJogSpeed;
    magazineMotor.set(amount);
  }
  private boolean positionRange(double input, double target){
    double high,low;
    high = Math.abs(target) + RobotMap.positionRangeModifier;
    low = Math.abs(target) - RobotMap.positionRangeModifier;
    if(Math.abs(input) < high && Math.abs(input) > low)
    return true;
    else 
    return false;
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
