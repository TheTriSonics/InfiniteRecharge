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
import frc.robot.*;

public class BallDeliverySubsystem extends SubsystemBase {
  VictorSPX ballDeliveryVictor;
  DigitalInput photoeyeTop, photoeyeBottom;

  public BallDeliverySubsystem() {
    ballDeliveryVictor = new VictorSPX(Constants.BALL_DELIVERY);
    photoeyeTop = new DigitalInput(Constants.PHOTOEYE_TOP);
    photoeyeBottom = new DigitalInput(Constants.PHOTOEYE_BOTTOM);
  }

  public void setPower(double power){
    ballDeliveryVictor.set(ControlMode.PercentOutput, power);
  }

  public boolean getTopPhotoeye(){
    return !photoeyeTop.get();
  }

  public boolean getBottomPhotoeye(){
    return !photoeyeBottom.get();
  }

  @Override
  public void periodic() {
    if (getTopPhotoeye()) {
      setPower(0);
      return;
    }
    if (Robot.robotState.isIntakeOn() || Robot.robotState.isShooterReady()) {
      setPower(1);
      return;
    }
    setPower(0);
    System.out.println("Bottom: " + getBottomPhotoeye() + "Top: " + getTopPhotoeye());
  }
}
