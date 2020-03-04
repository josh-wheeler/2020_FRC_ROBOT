/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class JogLiftCommand extends Command {

  /*
  
  This needs to be on a held down button so it cancels and stops. Otherwise it'll burn up the motor

  */
  private double amount;

  public JogLiftCommand(double amount) {

    this.amount = amount;
    // Use requires() here to declare subsystem dependencies
    requires(Robot.scissorLift);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //System.out.println("pushed joglift" + amount);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.scissorLift.jogLift(amount);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
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
