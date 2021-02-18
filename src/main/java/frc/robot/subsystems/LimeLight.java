/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase {
  NetworkTable table;
  NetworkTableEntry tx, ty, ta, tv;
  double x, y, area, seen;
  /**
   * Creates a new LimeLight.
   */
  public LimeLight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
  }
  public void readData(){
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");
    
    //read values periodically
    x = tx.getDouble(Double.NaN);
    y = ty.getDouble(Double.NaN);
    area = ta.getDouble(Double.NaN);
    seen = tv.getDouble(0);
  
    
    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("Limelight Target", seen);
  }
  public boolean isTargetSeen(){
    return seen>0.5;
  }
  public double getX(){
    return x;
  }
  public double getY(){
    return y;
  }

  public void setLEDState(boolean on) {
    table.getEntry("ledMode").setNumber(on ? 3 : 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
