/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class CHOOT extends Command {
  public CHOOT() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.shooterSubsystem);
    requires(Robot.ballMagazine);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.shooterSubsystem.startShooter();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {    //Robot.shooterSubsystem.startShooter();
    Robot.ballMagazine.CHOOT();
    //Robot.ballMagazine.revolve();;

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(Robot.ballMagazine.ballCounter() == 0)
    return true;
    else
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.shooterSubsystem.stopShooter();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
