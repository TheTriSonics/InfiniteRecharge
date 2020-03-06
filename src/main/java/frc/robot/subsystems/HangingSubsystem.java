/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class HangingSubsystem extends SubsystemBase {
  /**
   * Creates a new HangingSubsystem.
   */
  TalonSRX left, right;
  double leftOffset = 0;
  double rightOffset = 0;

  public HangingSubsystem() {
    left = new TalonSRX(Constants.LEFT_HANGING);
    right = new TalonSRX(Constants.RIGHT_HANGING);
    // left.setInverted(true);
    left.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    right.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    left.setSensorPhase(true);
  }

  public void setPower(double power){
    left.set(ControlMode.PercentOutput, power);
    right.set(ControlMode.PercentOutput, power);
  }

  public void setLeftPower(double power) {
    left.set(ControlMode.PercentOutput, power);
  }
  public void setRightPower(double power) {
    right.set(ControlMode.PercentOutput, power);
  }

  public void resetEncoders() {
    leftOffset = left.getSelectedSensorPosition();
    rightOffset = right.getSelectedSensorPosition();
  }

  public double getLeftPosition() {
    return left.getSelectedSensorPosition() - leftOffset;
  }
  public double getRightPosition() {
    return right.getSelectedSensorPosition() - rightOffset;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double leftPosition = getLeftPosition();
    double rightPosition = getRightPosition();
    if (Robot.robotState.isEndGame()){
      double power = - Robot.oi.operator.getY(Hand.kLeft);
      if ((leftPosition < 0 || rightPosition < 0) && power < 0 && !Robot.oi.operator.getAButton()) power = 0;
      if (Math.abs(leftPosition - rightPosition) > 150) {
        if (leftPosition > rightPosition) {
          if (power > 0) {
            setLeftPower(0.8*power);
            setRightPower(power);
          } else {
            setLeftPower(power);
            setRightPower(0.8*power);
          }
        } else {
          if (power > 0) {
            setLeftPower(power);
            setRightPower(0.8*power);
          } else {
            setLeftPower(0.8*power);
            setRightPower(power);
          }
        }
      } else setPower(power);
    }
  //   SmartDashboard.putNumber("left hanging", getLeftPosition());
  //   SmartDashboard.putNumber("right hanging", getRightPosition());
  }
}
