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

/**
 * Add your docs here.
 */
public class Limelight extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  NetworkTable llTable = NetworkTableInstance.getDefault().getTable("limelight");
  //private double leftTurn, rightTurn, steerAdjust = 0;

 
  public Limelight(){
    //sets camera mode to driver cam for initial startup
   SetTargetMode(false);

  }
  //returns current value for limelight variables
  public double tx()  {return llTable.getEntry("tx").getDouble(0);}
  public double ty()  {return llTable.getEntry("ty").getDouble(0);}
  public double tv()  {return llTable.getEntry("tv").getDouble(0);}
  public double ta()  {return llTable.getEntry("ta").getDouble(0);}
  public double ts()  {return llTable.getEntry("ts").getDouble(0);}
  public double camMode(){return llTable.getEntry("camMode").getDouble(0);}
  public double ledMode(){return llTable.getEntry("ledMode").getDouble(0);}

  public void LimelightUpdate(){

    SmartDashboard.putNumber("Y angle", ty());
    SmartDashboard.putNumber("X angle", tx());
    SmartDashboard.putNumber("Skew?", ts());
    SmartDashboard.putNumber("Target area", ta());
    SmartDashboard.putNumber("Target aquired", tv());

  }
  //sets the camera mode to Vision processor if true, driver cam if false
  public void SetTargetMode(boolean mode){

    if(mode){
      llTable.getEntry("camMode").setNumber(0);
    }
    else{
      llTable.getEntry("camMode").setNumber(1);
    }
  }
  public void targetModeToggle(){
  if(camMode()==1){
    SetTargetMode(true);
  }
  else{
    SetTargetMode(false);
  }

  }

  public void toggleLimelightLed(){
    if(ledMode()==1){
      setLed(3);
    }
    else{
      setLed(1);
    }
    
  }
  public void setLed(double mode){
    //0 is use pipeline default, 1 is force off, 2 is force blink, 3 is force on
      llTable.getEntry("ledMode").setNumber(mode);
    
  }
  public void AIM(){

    double steerAdjust = (RobotMap.aimIncrement * tx());
    SmartDashboard.putNumber("turnToTarget setting", steerAdjust);
    //checks to see if limelight has target
    if(tv() == 1){
      //deadzone for steering adjustment
      if(Math.abs(steerAdjust) > RobotMap.aimDeadzone){
        //sends adjusted limelight tx() to turnToTarget method on drive subsystem              
        Robot.driveSubsystem.turnToTarget(steerAdjust, -steerAdjust);   
      }
    }
  }



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
