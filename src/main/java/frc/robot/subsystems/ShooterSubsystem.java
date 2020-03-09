/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ShooterSubsystem extends Subsystem {

  CANSparkMax topShooterMotor = new CANSparkMax(RobotMap.topShooterMotorPort, RobotMap.NEO);
  CANSparkMax bottomShooterMotor = new CANSparkMax(RobotMap.bottomShooterMotorPort, RobotMap.NEO);
  CANEncoder topEncoder = topShooterMotor.getEncoder();
  CANEncoder bottomEncoder = bottomShooterMotor.getEncoder();
  CANPIDController topPID = topShooterMotor.getPIDController(); 
  CANPIDController bottomPID = bottomShooterMotor.getPIDController();

  RangeChecker rangeChecker = new RangeChecker();

  public boolean shooterOn;
  private double topTargetRPM, bottomTargetRPM;

  private double targetArea, targetAngle;
  
    //sets speed limits for AIM() method
    private double shooterMaxRPM = 4500;
    private double shooterMinRPM = 2000;


  private static double kP = 0.00002; // .5
  private static double kI = 0.00000015; // .0
  private static double kD = 0.0000; // .0
  private static double kIz = 0.0;
  private static double kFF = 0.0;
  private static double kMaxOutput = 1; 
  private static double kMinOutput = -1;

//constructor
  public ShooterSubsystem(){
    shooterOn = false;
    setTargets(0.0);
    topShooterMotor.setIdleMode(IdleMode.kCoast);
    bottomShooterMotor.setIdleMode(IdleMode.kCoast);
    topPID.setP(kP);
    topPID.setI(kI);
    topPID.setD(kD);
    topPID.setIZone(kIz);
    topPID.setFF(kFF);

    bottomPID.setP(kP);
    bottomPID.setI(kI);
    bottomPID.setD(kD);
    bottomPID.setIZone(kIz);
    bottomPID.setFF(kFF);

    topPID.setOutputRange(kMinOutput, kMaxOutput);
    bottomPID.setOutputRange(kMinOutput, kMaxOutput);
    //shooterPIDStatus();
  }

  //updates smartdashboard. called in robot's periodic method
  public void shooterStatus(){
    SmartDashboard.putNumber("Top Target Speed", topTargetRPM);
    SmartDashboard.putNumber("Bottom Target Speed", bottomTargetRPM);
    SmartDashboard.putNumber("Top Shooter Motor Speed", topEncoder.getVelocity());
    SmartDashboard.putNumber("Bottom Shooter Motor Speed", bottomEncoder.getVelocity());
    SmartDashboard.putBoolean("Shooter upToSpeed", upToSpeed());
    SmartDashboard.putBoolean("ShooterOn", shooterOn);
    SmartDashboard.putNumber("Target Area", targetArea);

    //ShooterMotorTuner();

  }

  //starts motors to currently set target RPMS.
  public void startShooter(){
    //topPID.setReference(topTargetRPM, ControlType.kVelocity);
    //bottomPID.setReference(bottomTargetRPM, ControlType.kVelocity);
    topShooterMotor.set(topTargetRPM/5676);
    bottomShooterMotor.set(bottomTargetRPM/5676);
    shooterOn = true;
  }

  public void inputTargetData(double ta, double ty){
    this.targetArea = ta;
    this.targetAngle = ty;
  }

  public void calcSpin(){
    //ty() range:-24.85 to 24.85
    //math for dist:  d = (heightoftarget-heightofcamera) / tan(angleofcamera + angletotarget)
    //limelight angle: 25 target height: 98.25 in (center of inside upper target) Limelight height: 22.25 in 
    //length of field: roughly 578. 
    //dividing by this gives us a percentage for the motors (if we are 578 inches from target, output = 1 full power)
    //double distanceToTarget = (76) / Math.tan(targetAngle+25);

    // math for figuring out numerator: k * math.sqrt(targetArea)
    double numerator = .5;                                                                   //<-------------------------adjust auto shooter speed here


    //math for distance to target (and by extension, shooter RPM) goes here. this is my white whale.
    double setting = numerator/Math.sqrt(targetArea);
    
    if(setting > shooterMaxRPM/5676)
      setting = shooterMaxRPM/5676; 
    else if(setting < shooterMinRPM/5676)
      setting = shooterMinRPM/5676;
    
    setTargets(setting);
  }

  //stops motors and sets target speeds to 0.0
  public void stopShooter(){   
    setTargets(0.0);
    topShooterMotor.set(0.0);
    bottomShooterMotor.set(0.0);
    shooterOn = false;
  }


  public void setTargets(double setting){

    //this converts duty cycle to RPM
      setting = setting * 5676;
      
      topTargetRPM = setting;
      bottomTargetRPM = -setting;
      
     
   }
 

  //this is for the ballMagazine, to tell it to only release balls when the motors are at speed.
  public boolean upToSpeed(){
    if(rangeChecker.within(topEncoder.getVelocity(), topTargetRPM, RobotMap.upToSpeedRange) && rangeChecker.within(bottomEncoder.getVelocity(), bottomTargetRPM, RobotMap.upToSpeedRange) && topTargetRPM != 0)
    return true;
    else
    return false;
  }

  
  public void ShooterMotorTuner(){
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    //double maxV = SmartDashboard.getNumber("Max Velocity", 0);
    //double minV = SmartDashboard.getNumber("Min Velocity", 0);
    //double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
    //double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);
 
    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { topPID.setP(p); kP = p; }
    if((i != kI)) { topPID.setI(i); kI = i; }
    if((d != kD)) { topPID.setD(d); kD = d; }
    if((iz != kIz)) { topPID.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { topPID.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
       topPID.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    //if((maxV != maxVel)) { masterPID.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    //if((minV != minVel)) { masterPID.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    //if((maxA != maxAccel)) { masterPID.setSmartMotionMaxAccel(maxA,0); maxAccel = maxA; }
   // if((allE != allowedErr)) { masterPID.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }
   }
   public void shooterPIDStatus(){
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
   //setDefaultCommand(new ShooterStopCommand());
  }
}
