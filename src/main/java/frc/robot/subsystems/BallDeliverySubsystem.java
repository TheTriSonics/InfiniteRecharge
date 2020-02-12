/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BallDeliverySubsystem extends SubsystemBase {
  VictorSPX ballDeliveryVictor;
  DigitalInput photoeyeTop, photoeyeBottom;

  public BallDeliverySubsystem() {
    ballDeliveryVictor = new VictorSPX(Constants.BALL_DELIVERY);
    photoeyeTop = new DigitalInput(Constants.PHOTOEYE_TOP);
    photoeyeBottom = new DigitalInput(Constants.PHOTOEYE_BOTTOM);
  }

  public void setBallDelliveryPower(double power){
    ballDeliveryVictor.set(ControlMode.PercentOutput, power);
  }

  public boolean getTopPhotoeye(){
    return photoeyeTop.get();
  }

  public boolean getBottomPhotoeye(){
    return photoeyeBottom.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
