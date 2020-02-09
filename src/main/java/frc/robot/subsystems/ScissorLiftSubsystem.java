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

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ScissorLiftSubsystem extends Subsystem {

  CANSparkMax leftLiftMotor = new CANSparkMax(RobotMap.leftLiftMotorPort, RobotMap.NEO);
  //CANSparkMax rightLiftMotor = new CANSparkMax(RobotMap.bottomShooterMotorPort, RobotMap.NEO);
  CANEncoder leftEncoder = leftLiftMotor.getEncoder();
  //CANEncoder rightEncoder = rightLiftMotor.getEncoder();
  CANPIDController leftPID = leftLiftMotor.getPIDController();
  //CANPIDController rightPID = rightLiftMotor.getPIDController();
  
  private static final double p = 0.00005; // .5
  private static final double i = 0.000001; // .0
  private static final double d = 0.0; // .0
  private static final double ff = .00015;
  private static final double maxOutputRange = 1; 
  private static final double minOutputRange = -1;


//constructor
  public ScissorLiftSubsystem(){

    leftLiftMotor.setIdleMode(IdleMode.kBrake);
    //rightLiftMotor.setIdleMode(IdleMode.kBrake);
    leftPID.setP(p);
    leftPID.setI(i);
    leftPID.setD(d);
    leftPID.setFF(ff);
    /*
    rightPID.setP(p);
    rightPID.setI(i);
    rightPID.setD(d);
    rightPID.setFF(ff);
    */
    leftPID.setOutputRange(minOutputRange, maxOutputRange);
  }

  //updates smartdashboard. called in robot's periodic method
  public void liftStatus(){
    SmartDashboard.putNumber("Lift Motor Speed", leftEncoder.getVelocity());
  }

  //starts motors to currently set target RPMS.
  public void startLift(double rate){
    //leftPID.setReference(topTargetRPM, ControlType.kVelocity);
    //rightPID.setReference(bottomTargetRPM, ControlType.kVelocity);

    //-1 is up, +1 is down

    leftLiftMotor.set(rate);
    //bottomShooterMotor.set(bottomTargetRPM/5676);
  }

  //stops motors and sets target speeds to 0.0
  public void stopLift(){   
    leftLiftMotor.set(0.0);
  }
 

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
   //setDefaultCommand(new ShooterStopCommand());
  }
}
