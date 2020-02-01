/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ShooterSubsystem extends Subsystem {

  CANSparkMax topShooterMotor = new CANSparkMax(RobotMap.topShooterMotorPort, RobotMap.topShooterMotorType);
  CANSparkMax bottomShooterMotor = new CANSparkMax(RobotMap.bottomShooterMotorPort, RobotMap.bottomShooterMotorType);
  CANEncoder topEncoder = topShooterMotor.getEncoder();
  CANEncoder bottomEncoder = bottomShooterMotor.getEncoder();
  private boolean shooterOn;
  private double topTargetSpeed, bottomTargetSpeed;

//constructor
  public ShooterSubsystem(){
    shooterOn = false;
 
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public boolean shooterStatus(){
    return shooterOn;
  }

  //starts motors and sets target speeds to whatever input the command specifies
  public void setShooter(double topMotorSpeed, double bottomMotorSpeed){
   setTargets(topMotorSpeed, bottomMotorSpeed);
   topShooterMotor.set(topMotorSpeed);
   bottomShooterMotor.set(bottomMotorSpeed);
   shooterOn = true;
   SmartDashboard.putNumber("Top Shooter Motor Speed", Robot.shooterSubsystem.getTopMotorSpeed());
   SmartDashboard.putNumber("Bottom Shooter Motor Speed", Robot.shooterSubsystem.getBottomMotorSpeed());
   SmartDashboard.putBoolean("Shooter upToSpeed", Robot.shooterSubsystem.upToSpeed());
  }

  //stops motors and sets target speeds to 0.0
  public void stopShooter(){   
    setTargets(0.0, 0.0);
    topShooterMotor.set(0.0);
    bottomShooterMotor.set(0.0);
    shooterOn = false;
    SmartDashboard.putNumber("Top Shooter Motor Speed", Robot.shooterSubsystem.getTopMotorSpeed());
    SmartDashboard.putNumber("Bottom Shooter Motor Speed", Robot.shooterSubsystem.getBottomMotorSpeed());
    SmartDashboard.putBoolean("Shooter upToSpeed", Robot.shooterSubsystem.upToSpeed());
  }

  //this will be for the conveyor, to tell it to only release balls when the motors are at speed.
  public boolean upToSpeed(){
    if(speedRange(topEncoder.getVelocity()) && speedRange(bottomEncoder.getVelocity()))
    return true;
    else
    return false;
  }

  public double getTopMotorSpeed(){
    //return topShooterMotor.get();
    return topEncoder.getVelocity();
  }

  public double getBottomMotorSpeed(){
    //return bottomShooterMotor.get();
    return bottomEncoder.getVelocity();
  }

  //sets target speeds for motors whenever the startShooter() or stopShooter() methods are called
  private void setTargets(double topMoto, double bottomMoto){
    topTargetSpeed = topMoto;
    bottomTargetSpeed = bottomMoto;
  }

  public boolean speedRange(double input){
    double high,low;
    high = Math.abs(topTargetSpeed) + RobotMap.upToSpeedRange;
    low = Math.abs(bottomTargetSpeed) - RobotMap.upToSpeedRange;
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
