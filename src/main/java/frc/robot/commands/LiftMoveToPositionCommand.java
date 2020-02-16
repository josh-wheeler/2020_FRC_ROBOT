/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ScissorLiftSubsystem.liftPosition;

public class LiftMoveToPositionCommand extends Command {

  private liftPosition position;

  public LiftMoveToPositionCommand(liftPosition position) {
    this.position = position;
    // Use requires() here to declare subsystem dependencies
    requires(Robot.scissorLift);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //add encoderZero check here
    Robot.scissorLift.moveToPosition(position);


  }
 
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //Robot.scissorLift.startLift(amount);
   Robot.scissorLift.LiftMotorTuner();

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //return Robot.scissorLift.homedOut;
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.scissorLift.stopLift();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
