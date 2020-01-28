/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.HumanDriveCommand;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem {

  // instantiate new motor controller objects

  CANSparkMax leftMaster = new CANSparkMax(RobotMap.leftMasterPort, RobotMap.leftMasterMotorType);
  CANSparkMax leftSlave = new CANSparkMax(RobotMap.leftSlavePort, RobotMap.leftSlaveMotorType);
  CANSparkMax rightMaster = new CANSparkMax(RobotMap.rightMasterPort, RobotMap.rightMasterMotorType);
  CANSparkMax rightSlave = new CANSparkMax(RobotMap.rightSlavePort, RobotMap.rightSlaveMotorType);
 
  // instantiate a new DifferentialDrive object pass motor controllers as arguments
  public DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);
  
  // constant multiplier to slow turn rate
  //private double turnMultiplier = .5;
 
  // create constructor function

  public DriveSubsystem(){
  
    // point slaves to masters
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

  }


  // add manualDrive() method

  public void humanDrive(double move, double turn){


    if (Math.abs(move) < 0.10) {				
      move = 0;
    }
    if (Math.abs(turn) < 0.30) {
      turn = 0;
    }
    move = Robot.motorNanny.newSpeed(move);

    drive.arcadeDrive(move, turn*RobotMap.turnMultiplier);

  }
  
  // Put methods for controlling this subsystem
  // here. Call these from Commands.


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new HumanDriveCommand());
  }
}
