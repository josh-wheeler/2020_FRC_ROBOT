/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.LimelightDriverCamCommand;
import frc.robot.commands.LimelightTargetCamCommand;

/**
 * Add your docs here.
 */
public class LimelightSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  public NetworkTable llTable = NetworkTableInstance.getDefault().getTable("limelight");
  


  public enum camMode{
    driver, target
  }
  public enum ledMode{
    def, off, blink, on
  }

  public LimelightSubsystem(){
    //sets camera mode to driver cam and led off for initial startup
   //setCamMode(camMode.driver);
   //setLed(ledMode.off);
  }
  //returns current value for limelight variables
  public double tx()  {return llTable.getEntry("tx").getDouble(0);}
  public double ty()  {return llTable.getEntry("ty").getDouble(0);}
  public double tv()  {return llTable.getEntry("tv").getDouble(0);}
  public double ta()  {return llTable.getEntry("ta").getDouble(0);}
  public double ts()  {return llTable.getEntry("ts").getDouble(0);}
  public double camMode(){return llTable.getEntry("camMode").getDouble(0);}
  public double ledMode(){return llTable.getEntry("ledMode").getDouble(0);}

  //posts smartdashboard values
  public void LimelightUpdate(){

    SmartDashboard.putNumber("Y angle", ty());
    SmartDashboard.putNumber("X angle", tx());
    //SmartDashboard.putNumber("Skew?", ts());
    SmartDashboard.putNumber("Target area", ta());
    SmartDashboard.putNumber("Target aquired", tv());
    //SmartDashboard.putNumber("Distance to Target", distanceToTarget);

  }

  //sets the camera mode to Vision processor if true, driver cam if false
  public void setCamMode(camMode mode){

    switch(mode){
      case driver:
      llTable.getEntry("camMode").setNumber(1);
      break;
      case target:
      llTable.getEntry("camMode").setNumber(0);
      break;
    }
  }
    
  
  public void setLed(ledMode mode){
    //0 is use pipeline default, 1 is force off, 2 is force blink, 3 is force on
      switch(mode){
        case def:
        llTable.getEntry("ledMode").setNumber(0);
        break;
        case off:
        llTable.getEntry("ledMode").setNumber(1);
        break;
        case blink:
        llTable.getEntry("ledMode").setNumber(2);
        break;
        case on:
        llTable.getEntry("ledMode").setNumber(3);
        break;
      }
    
  }

    //constants for AIM method
    //math for dist:  d = (heightoftarget-heightofcamera) / tan(angleofcamera + angletotarget)
    //length of field: roughly 578. 
    //dividing by this gives us a percentage for the motors (if we are 578 inches from target, output =1 full power)

  public double AIM(){
    //this gets changed to a speed constant in the turnToTarget method. the direction is passed to get the sign (for direction, obv)
    double direction = tx();
    if(tv() == 1.0 ){
      if(Math.abs(direction) > RobotMap.aimDeadZone)
      return direction;
      else
      return 0.0;
    }
    else{
      return 0.4;
      //returning .4 as a default if there is no target allows us to slowly scan right and look for a target
    }

  }

  public void outputTargetData(){
    Robot.shooterSubsystem.inputTargetData(ta(),ty());
  }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new LimelightDriverCamCommand());
    //setDefaultCommand(new LimelightTargetCamCommand());

  }
}
