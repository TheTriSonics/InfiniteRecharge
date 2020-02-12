/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ColorWheelRotateSubsystem extends SubsystemBase {
  CANSparkMax rotateWheel;
  CANEncoder encoder;
  
  public ColorWheelRotateSubsystem() {
    rotateWheel = new CANSparkMax(Constants.CONTROL_WHEEL, MotorType.kBrushless);
    rotateWheel.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    encoder = rotateWheel.getEncoder();
  }

  public void setPower(double power){
    rotateWheel.set(power);
  }

  public void extendControlWheel(boolean extend){
    Robot.pneumatics.setState(Pneumatics.CONTROL_WHEEL, extend);
  }

  public double getPosition(){
    return encoder.getPosition();
  }

  public void displayEncoder(){
    SmartDashboard.putNumber("Encoder:", rotateWheel.getEncoder().getPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //setPower(Robot.oi.driver.getY(Hand.kLeft));
    displayEncoder();
  }
}
