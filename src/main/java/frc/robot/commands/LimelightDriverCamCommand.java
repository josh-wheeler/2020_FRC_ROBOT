/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.LimelightSubsystem.camMode;
import frc.robot.subsystems.LimelightSubsystem.ledMode;

/**
 * Add your docs here.
 */
public class LimelightDriverCamCommand extends InstantCommand {
  /**
   * Add your docs here.
   */


  public LimelightDriverCamCommand() {
    super();
    
    // Use requires() here to declare subsystem dependencies
    requires(Robot.limelight);
  }

  // Called once when the command executes 
  @Override
  protected void initialize() {
    Robot.limelight.setCamMode(camMode.driver);
    Robot.limelight.setLed(ledMode.off);
    //Robot.limelight.llTable.getEntry("camMode").setNumber(1);
    //Robot.limelight.llTable.getEntry("ledMode").setNumber(0);

  }

}
