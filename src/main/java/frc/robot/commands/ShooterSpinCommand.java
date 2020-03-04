/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
/*


This command can be called three ways. 
No arguments () makes the speed the default, 
1 argument (double) set the speeds, 

it then passes these to the setTargets() method in the shooter subsystem, and calls startShooter()

*/ 
public class ShooterSpinCommand extends Command {
  private double speed;

  public ShooterSpinCommand(){
    this.speed = RobotMap.ShooterDefaultSpeed;
    requires(Robot.shooterSubsystem);
 }

  public ShooterSpinCommand(double input){
    this.speed = input;
    requires(Robot.shooterSubsystem);
  }


  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //Robot.shooterSubsystem.setTargets(speed); 
    Robot.shooterSubsystem.startShooter();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //Robot.shooterSubsystem.startShooter();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
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
