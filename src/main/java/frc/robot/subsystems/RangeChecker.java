package frc.robot.subsystems;

public class RangeChecker {

  public boolean within(double var, double target, double range){
    if(Math.abs(var) < Math.abs(var) + range && Math.abs(var) > Math.abs(var) - range)    
    return true;
    else 
    return false;
  }
}
