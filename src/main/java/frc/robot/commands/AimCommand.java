/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.LimelightSubsystem.camMode;
import frc.robot.subsystems.LimelightSubsystem.ledMode;

public class AimCommand extends Command {
  public AimCommand() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveSubsystem);
    requires(Robot.limelight);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.limelight.setCamMode(camMode.target);
    Robot.limelight.setLed(ledMode.on); 
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
   Robot.driveSubsystem.turnToTarget(Robot.limelight.AIM());
   //Robot.limelight.calcShooterSpeed();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //if(Robot.limelight.AIM() == 0.0)
      //return true;
    //else
      return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //Robot.limelight.setCamMode(camMode.driver);
    //Robot.limelight.setLed(ledMode.off);
    Robot.driveSubsystem.turnToTarget(0.0);
    Robot.limelight.calcShooterSpeed();

    
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
