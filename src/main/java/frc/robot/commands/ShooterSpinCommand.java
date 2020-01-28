/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ShooterSpinCommand extends Command {
  private double topSpd, bottomSpd;

 
  public ShooterSpinCommand(double topSpd, double bottomSpd) {
    this.topSpd = topSpd;
    this.bottomSpd = bottomSpd;
    // Use requires() here to declare subsystem dependencies
    requires(Robot.shooterSubsystem);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.shooterSubsystem.startShooter(topSpd, bottomSpd);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //return Robot.shooterSubsystem.getStandby();
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //Robot.shooterSubsystem.stopShooter();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}