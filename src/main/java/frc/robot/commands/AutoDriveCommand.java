/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class AutoDriveCommand extends Command {

  private int timer = 200;
  private int turnTime = 125;
  private double speed = -.35;

  public AutoDriveCommand() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //Robot.driveSubsystem.drive.tankDrive(speed, speed);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    timer--;
    if(timer > turnTime)
    Robot.driveSubsystem.drive.tankDrive(speed, speed);
    else
    Robot.driveSubsystem.drive.tankDrive(speed,-speed);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(timer == 0)
    return true;
    else
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveSubsystem.drive.tankDrive(0.0, 0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
