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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.HumanDriveCommand;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem {

  // instantiate new motor controller objects
  CANSparkMax leftMaster = new CANSparkMax(RobotMap.leftMasterPort, RobotMap.NEO);
  CANSparkMax leftSlave = new CANSparkMax(RobotMap.leftSlavePort, RobotMap.NEO);
  CANSparkMax rightMaster = new CANSparkMax(RobotMap.rightMasterPort, RobotMap.NEO);
  CANSparkMax rightSlave = new CANSparkMax(RobotMap.rightSlavePort, RobotMap.NEO);
  CANEncoder rightEncoder = rightMaster.getEncoder();
  CANEncoder leftEncoder  = leftMaster.getEncoder();
  // instantiate a new DifferentialDrive object and pass motor controllers as arguments
  public DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);
  
  //current speed for acceleratorControl method
  double currSpeed = 0.0;

  public DriveSubsystem(){
  
    // point slaves to masters
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

  }

  // add manualDrive() method

  public void humanDrive(double move, double turn){

    //for joystick deadspots(less touchy conrols)
    if (Math.abs(move) < 0.10) {				
      move = 0;
    }
    if (Math.abs(turn) < 0.30) {
      turn = 0;
    }

    //modifies the joystick inputs to smooth them out and passes hem to the drive to move it
    drive.arcadeDrive(acceleratorControl(move), turn*RobotMap.turnMultiplier);
    //should show current set speed on smartdashboard (the move variable after filtering)
    SmartDashboard.putNumber("Current Set Drive Speed", currSpeed);

    //shows left and right drivetrain velocitys
    SmartDashboard.putNumber("Right Master Drive Velocity", rightEncoder.getVelocity());
    SmartDashboard.putNumber("Left Master Drive Velocity", leftEncoder.getVelocity());

  }
  
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

public void turnToTarget(double rightTurn, double leftTurn){
  //this will turn the robot when the AIM() method is called on the limelight. It will probably need to be filtered to go slowly
  drive.tankDrive(leftTurn, rightTurn);
}
  //for smoothing acceleration. It uses a maxAccel variable to use the periodic function timing to slowly increase robot speed. 
  //i.e. periodic calls every 20ms, the robot's speed can only increase by the maxAccel (.02) every 20ms. prevents jerky robot motion
  private double acceleratorControl(double newSpeed){
   
    double sign = 0;

    if (newSpeed > currSpeed){
      sign = 1.0;
    }
    else{
      sign = -1.0;
    }

    double deltaSpeed = Math.abs(currSpeed - newSpeed);

    if (deltaSpeed > RobotMap.maxAccel){
      deltaSpeed = RobotMap.maxAccel;
    }

    currSpeed = currSpeed + (sign *deltaSpeed);

    if (currSpeed > 0){
      sign = 1; 
    }
    else{ 
      sign = -1;
    }
    
    if (Math.abs(currSpeed) > RobotMap.maxSpeed){
      currSpeed = sign * RobotMap.maxSpeed;
    }

      return currSpeed;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new HumanDriveCommand());
  }
}
