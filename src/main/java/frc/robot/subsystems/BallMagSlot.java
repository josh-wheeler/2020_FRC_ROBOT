/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

/**
 * Add your docs here.
 */
public class BallMagSlot {

    public boolean ballPresent;
    public boolean atLoadPos;
    public BallMagSlot(){
        ballPresent = false;
        
    }
    public void setAtLoadPosition(){
        setBallPresent(false);
        setLoadposition(true);
    }
    public void setBallLoaded(){
        setBallPresent(true);;
        //setLoadposition(false);
    }
    public void atFirePosition(){

    }
    public void setBallPresent(boolean ballpresent){
        this.ballPresent = ballpresent;
    }
    public void setLoadposition(boolean atLoadPos){
        this.atLoadPos = atLoadPos;
    }
}


