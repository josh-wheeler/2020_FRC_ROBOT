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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.JogMagCommand;
import frc.robot.commands.MagazinePIDCommand;

/**
 * Add your docs here.
 */
public class BallMagazineSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  CANSparkMax magazineMotor = new CANSparkMax(RobotMap.magazineMotorPort, RobotMap.neo550);
  CANEncoder magazineEncoder = magazineMotor.getEncoder();
  CANPIDController magazinePID = magazineMotor.getPIDController();



  public BallMagSlot BS1 = new BallMagSlot();
  public BallMagSlot BS2 = new BallMagSlot();
  public BallMagSlot BS3 = new BallMagSlot();

  //number of rotations required to advance the ball 120°. 
  private double rotateAmount = -21;
  //number to add rotateAmount to so motor always travels in one direction.
  private double currentSetPosition = 0;

  public int ballCount = 0;

  public boolean active;

  private double kP = .01; // .0095
  private double kI = 15e-6; // .000015
  private double kD = 0.2;//0.000000001;
  private double kIz = 0.0;
  private double kFF = 0.0;
  private double kMaxOutput = 1.0; 
  private double kMinOutput = -1.0;



//-----------------------------------------

  public BallMagazineSubsystem(){
    active = false;

    magazineMotor.restoreFactoryDefaults();

    magazineMotor.setIdleMode(IdleMode.kBrake);
    magazinePID.setP(kP);
    magazinePID.setI(kI);
    magazinePID.setD(kD);
    magazinePID.setIZone(kIz);
    magazinePID.setFF(kFF);
    magazinePID.setOutputRange(kMinOutput, kMaxOutput);

    //this zeros the motor encoder (we use the motor encoder to PID the magazine to position)
    magazineEncoder.setPosition(0.0);


    /*
    //the next lines set the ball present for competition pre-loading.     
    BS1.ballPresent=true;
    BS2.ballPresent=true;
    BS3.ballPresent=true;
    */


    BS1.atLoadPos = true;
    ballCounter();
    magPIDStatus();
  }

  public void magPIDPosition(){
    //this is constantly running to keep the mag at whatever the currentSetPosition is
    magazinePID.setReference(currentSetPosition, ControlType.kPosition);
  }

  public void zeroSetposition(){
    currentSetPosition = 0.0;
  }

  public void revolve(){
        
    //this subrtracts 120° of movement each time, so the motor only goes one direction.
    currentSetPosition = currentSetPosition + rotateAmount;

      if(BS1.atLoadPos){
        BS1.atLoadPos = false;
        BS2.setAtLoadPosition();
      }
      else if(BS2.atLoadPos){
        BS2.atLoadPos = false;
        BS3.setAtLoadPosition();
      }      
      else if(BS3.atLoadPos){
        BS3.atLoadPos = false;
        BS1.setAtLoadPosition();
      }


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
    SmartDashboard.putNumber("magazine setting", currentSetPosition);
    SmartDashboard.putNumber("magazine position", magazineEncoder.getPosition());
    SmartDashboard.putNumber("BALLS", ballCount);

    SmartDashboard.putBoolean("BS1 at load", BS1.atLoadPos);
    SmartDashboard.putBoolean("BS2 at load", BS2.atLoadPos);
    SmartDashboard.putBoolean("BS3 at load", BS3.atLoadPos);
    SmartDashboard.putBoolean("BS1 ball present", BS1.ballPresent);
    SmartDashboard.putBoolean("BS2 ball present", BS2.ballPresent);
    SmartDashboard.putBoolean("BS3 ball present", BS3.ballPresent);

  }

  public void CHOOT(){
    if(Robot.shooterSubsystem.upToSpeed()){
      if(BS1.ballPresent || BS2.ballPresent || BS3.ballPresent){
        revolve();
      }
    }
  }

  //this checks to see how many balls we have inside (that's what she said!)
  public void ballCounter(){
    int BC = 0;
    if(BS1.ballPresent)
    BC++;
    if(BS2.ballPresent)
    BC++;
    if(BS3.ballPresent)
    BC++;
    ballCount = BC;
  }


  //this is just for the jog function
  public boolean toggleActive(){
    active = !active;
    return active;
  }
  

  public void jogMag(){
    if(active)
    magazineMotor.set(-RobotMap.magJogSpeed);
    else
    magazineMotor.set(0.0);
  }
  public void stopMag(){
    magazineMotor.set(0.0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new MagazinePIDCommand());
  }

  public void magPIDStatus(){
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
   
    SmartDashboard.putNumber("Motor Output", magazineMotor.getAppliedOutput());

  }

  public void MagMotorTuner(){
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { magazinePID.setP(p); kP = p; }
    if((i != kI)) { magazinePID.setI(i); kI = i; }
    if((d != kD)) { magazinePID.setD(d); kD = d; }
    if((iz != kIz)) { magazinePID.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { magazinePID.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
       magazinePID.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
   
   }
 



}
