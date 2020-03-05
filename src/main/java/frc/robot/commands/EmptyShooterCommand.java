/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class EmptyShooterCommand extends Command {
  private int timer = 0;
  private int timerGap = 50;

  public EmptyShooterCommand(){
    // Use requires() here to declare subsystem dependencies
    requires(Robot.ballMagazine);
    requires(Robot.shooterSubsystem); 
  }

  public EmptyShooterCommand(double speed) {    
    // Use requires() here to declare subsystem dependencies
    requires(Robot.ballMagazine);
    requires(Robot.shooterSubsystem);
    Robot.shooterSubsystem.setTargets(speed);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.shooterSubsystem.startShooter();

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    timer++;
    if(timer == timerGap || timer == timerGap*2 || timer == timerGap*3){
      Robot.ballMagazine.revolve();
    }


    Robot.ballMagazine.magPIDPosition();
    
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(timer == timerGap*4)
    return true;
    else
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.ballMagazine.stopMag();
    //Robot.ballMagazine.resetMagazineEncoder();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
