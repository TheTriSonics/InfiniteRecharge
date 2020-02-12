/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class HangerSubsystem extends SubsystemBase {
  VictorSPX hangerVictor1, hangerVictor2;

  public HangerSubsystem() {
    hangerVictor1 = new VictorSPX(Constants.HANGING_MOTOR1);
    hangerVictor2 = new VictorSPX(Constants.HANGING_MOTOR2);
  }

  public void setExtend(boolean extend){
    Robot.pneumatics.setState(Pneumatics.HANGING_TILT, extend);
  }

  public void setLock(boolean lock){
    Robot.pneumatics.setState(Pneumatics.HANGING_LOCK, lock);
  }

  public void setPower(double power) {
    hangerVictor1.set(ControlMode.PercentOutput, power);
    hangerVictor2.set(ControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
