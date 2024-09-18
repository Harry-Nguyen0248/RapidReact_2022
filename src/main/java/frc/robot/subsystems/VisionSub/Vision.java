package frc.robot.subsystems.VisionSub;


import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants; 

import frc.robot.util.OrbitPID;


public class Vision extends SubsystemBase {

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tvert = table.getEntry("tvert");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry ledMode = table.getEntry("ledMode");

  private List<Double> tyList = new ArrayList<>();
  private final double tyLength = 10;



  public Vision() { 
   // setLLLed(false);
  }   
/*
  public void setLLLed(boolean set) {
    ledMode.setNumber(set? 3 : 1); //3 = led on; 1 = led off
  }
  */

  public boolean isTargetFound() { 
    return this.tv.getDouble(0.0) == 1.0; 
  } 

  public boolean isLedOn() {
    return ledMode.getNumber(0).intValue()==3;
  }
  
  public double getX() { 
    return this.tx.getDouble(0.0); 
  }
  public double getY() {
    return this.ty.getDouble(0.0);  
  }  

  public double getVertHeight() {
    return tvert.getDouble(Double.NaN);
  }

  // Advanced methods of calculations / robot actions and movement  

  public double getDistanceFromTarget() { 
    double angleFromGroundToTargetTop = Constants.Vision.cameraMountAngle + getY(); 
    double distance = (Constants.Vision.targetStructureHeight - Constants.Vision.cameraMountHeight) / Math.tan(Math.toRadians(angleFromGroundToTargetTop)); 
    return distance;  
  } 
  
  public double getAlignAdjustValue() { 
    // With PID Control altered values 
    double errorFromTarget = getX(); 

    OrbitPID alignment = new OrbitPID(Constants.Vision.targetAlignmentGains); 
    // Target is 0 (robot is fully aligned) and input is the current deviation 
    double adjustValue = alignment.calculate(0, errorFromTarget);  

    return adjustValue; 
  }

  public void setLLLed(boolean set) {
    ledMode.setNumber(set? 3: 1);
  }

  public double getAvgY(){
    int totalY = 0;
    for (int i = 0; i<tyList.size(); i++) {
      totalY += tyList.get(i);
    }
    double avgY = totalY/Constants.executeDuration * tyLength;
    SmartDashboard.putNumber("avgtY", avgY);
    return avgY;
  }

  @Override
  public void periodic() {
    // Record ty history
    tyList.add(getY());
    while (tyList.size() > tyLength) {
      tyList.remove(0);
    }

    SmartDashboard.putNumber("ty", getY());
    SmartDashboard.putNumber("tx", getX());
  }

}
