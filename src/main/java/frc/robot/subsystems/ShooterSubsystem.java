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

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ShooterSubsystem extends Subsystem {

  CANSparkMax topShooterMotor = new CANSparkMax(RobotMap.topShooterMotorPort, RobotMap.NEO);
  CANSparkMax bottomShooterMotor = new CANSparkMax(RobotMap.bottomShooterMotorPort, RobotMap.NEO);
  CANEncoder topEncoder = topShooterMotor.getEncoder();
  CANEncoder bottomEncoder = bottomShooterMotor.getEncoder();
  CANPIDController topPID = topShooterMotor.getPIDController();
  CANPIDController bottomPID = bottomShooterMotor.getPIDController();
  public boolean shooterOn;
  private double topTargetRPM, bottomTargetRPM;

//constructor
  public ShooterSubsystem(){
    shooterOn = false;
    setTargets(0.0, 0.0);
  }

  //updates smartdashboard. called in robot's periodic method
  public void shooterStatus(){
    SmartDashboard.putNumber("Top Target Speed", topTargetRPM);
    SmartDashboard.putNumber("Bottom Target Speed", bottomTargetRPM);
    SmartDashboard.putNumber("Top Shooter Motor Speed", topEncoder.getVelocity());
    SmartDashboard.putNumber("Bottom Shooter Motor Speed", bottomEncoder.getVelocity());
    SmartDashboard.putBoolean("Shooter upToSpeed", upToSpeed());
  }

  //starts motors to currently set targets.
  public void startShooter(){
    topPID.setReference(topTargetRPM, ControlType.kVelocity);
    bottomPID.setReference(bottomTargetRPM, ControlType.kVelocity);
    shooterOn = true;
  }

  //stops motors and sets target speeds to 0.0
  public void stopShooter(){   
    setTargets(0.0, 0.0);
    topShooterMotor.set(0.0);
    bottomShooterMotor.set(0.0);
    shooterOn = false;
  }
  //sets target speeds for motors
  public void setTargets(double topSetting, double bottomSetting){
     topTargetRPM = topSetting * 5676;
     bottomTargetRPM = bottomSetting * (-5676); 
   }
 

  //this will be for the conveyor, to tell it to only release balls when the motors are at speed.
  public boolean upToSpeed(){
    if(speedRange(topEncoder.getVelocity(),topTargetRPM) && speedRange(bottomEncoder.getVelocity(), bottomTargetRPM))
    return true;
    else
    return false;
  }


  private boolean speedRange(double input, double targetRPM){
    double high,low;
    high = Math.abs(targetRPM) + RobotMap.upToSpeedRange;
    low = Math.abs(targetRPM) - RobotMap.upToSpeedRange;
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
