/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class MotorNanny {
    
    //MathUtil.clamp(currSpeed, -1.0, 1.0);
    //Math.copySign(newSpeedValue*2, newSpeedValue);
    double currSpeed = 0.0;
    public double newSpeed(double newSpeedValue){
       
        double sign = 0;
        if (newSpeedValue > currSpeed){
            sign = 1.0;
        }
        else{
            sign = -1.0;
        }

        double deltaSpeed = Math.abs(currSpeed - newSpeedValue);

        if (deltaSpeed > RobotMap.maxAccel){
            deltaSpeed = RobotMap.maxAccel;
        }
        currSpeed = currSpeed + sign *deltaSpeed;
        if (currSpeed > 0){
            sign = 1; 
        }
        else{ 
            sign = -1;
        }
        if (Math.abs(currSpeed) > RobotMap.maxSpeed){
            currSpeed = sign * RobotMap.maxSpeed;
        }
        return currSpeed;
    }
   
}
