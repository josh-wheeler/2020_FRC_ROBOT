/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.IntakeCommand;

/**
 * Add your docs here.
 */
public class IntakeSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
CANSparkMax intakeRollerMotor = new CANSparkMax(RobotMap.intakeRollerMotorPort,RobotMap.neo550);
CANSparkMax transferConveyorMotor = new CANSparkMax(RobotMap.transferConveyorMotorPort,RobotMap.neo550);
//CANEncoder intakeEncoder = intakeRollerMotor.getEncoder();
//CANEncoder transferEncoder = transferConveyorMotor.getEncoder();

public DigitalInput transferBeam = new DigitalInput(RobotMap.transferBeamBreakPort);
//public DigitalInput magLimitSwitch = new DigitalInput(RobotMap.magBeamBreakPort);
private boolean active;

public IntakeSubsystem(){
  //intakeRollerMotor.restoreFactoryDefaults();
  //transferConveyorMotor.restoreFactoryDefaults();
  intakeRollerMotor.setIdleMode(IdleMode.kBrake);
  transferConveyorMotor.setIdleMode(IdleMode.kBrake);
  active = false;
}

public void ballIntake(){

  if(active){
    if(!transferBeam.get()){
      intakeOn(false);
      if(Robot.ballMagazine.readyToLoad()){
        loadMag();
      }
    
    }
    else {
      intakeOn(true);
    }
  }
  else{
    intakeOn(false);
  }
}
public void loadMag(){
  
  intakeOn(true);
  Robot.ballMagazine.loadBall();
  
}

public void intakeOn(boolean on){
  if(on){
    intakeRollerMotor.set(-RobotMap.intakeJogSpeed);
    transferConveyorMotor.set(-RobotMap.intakeJogSpeed);
  }
  else{
    intakeRollerMotor.set(0.0);
    transferConveyorMotor.set(0.0);
  }
}

public boolean toggleActive(){
  active = !active;
  return active;
}
public void intakeStatus(){
  SmartDashboard.putNumber("intake roller setting", intakeRollerMotor.get());
  SmartDashboard.putNumber("transfer conveyor setting", transferConveyorMotor.get());
  SmartDashboard.putBoolean("Beam Connected", transferBeam.get());
  //SmartDashboard.putBoolean("Magazine Switch OPEN", magLimitSwitch.get());
  
  SmartDashboard.putBoolean("Intake Active", active);

}
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new IntakeCommand());
  }
}
