/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.*;
import frc.robot.utilities.LEDMode;

public class BallDeliverySubsystem extends SubsystemBase {
  VictorSPX ballDeliveryVictor1, ballDeliveryVictor2;
  // AnalogInput photoeyeTop;
  // AnalogInput photoeyeBottom;

  public BallDeliverySubsystem() {
    ballDeliveryVictor1 = new VictorSPX(Constants.BALL_DELIVERY1);
    ballDeliveryVictor2 = new VictorSPX(Constants.BALL_DELIVERY2);
    // photoeyeTop = new AnalogInput(1);
    // photoeyeBottom = new AnalogInput(Constants.PHOTOEYE_BOTTOM);
  }

  public void setPower(double power){
    ballDeliveryVictor1.set(ControlMode.PercentOutput, power);
    ballDeliveryVictor2.set(ControlMode.PercentOutput, -power);
  }

  public boolean getTopPhotoeye(){
    return Robot.photoEyes.getTopPhotoEye();
    // return photoeyeTop.getVoltage() > 0.83;
  }

  public boolean getBottomPhotoeye(){
    return Robot.photoEyes.getBottomPhotoEye();
    // return photoeyeBottom.getVoltage() > 0.83;
  }

  @Override
  public void periodic() {
    if (Robot.robotState.isShooterReady()) {
      setPower(1);
      return;
    }
    
    if (Robot.robotState.isIntakeOn() && getTopPhotoeye() == false) {
      setPower(1);
      return;
    }

    setPower(0);
    SmartDashboard.putBoolean("Photoeye Top: ", getTopPhotoeye());
    SmartDashboard.putBoolean("Photoeye Bottom: ", getBottomPhotoeye());
    SmartDashboard.putNumber("Photoeye Top V: ", Robot.photoEyes.getTopVoltage());
    SmartDashboard.putNumber("Photoeye Bottom V: ", Robot.photoEyes.getBottomVoltage());
    // System.out.println("Bottom: " + getBottomPhotoeye() + " Top: " + getTopPhotoeye() + " | " + photoeyeTop.getVoltage());
  }
}
