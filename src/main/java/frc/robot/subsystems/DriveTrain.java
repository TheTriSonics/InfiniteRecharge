/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  TalonSRX leftFront, leftRear, rightRear, rightFront;
  Encoder leftDriveEncoder, rightDriveEncoder;

  double xLimit = 1;
  double yLimit = 1;
  int leftDriveOffset = 0, rightDriveOffset = 0;
  double leftDistanceOffset = 0, rightDistanceOffset = 0;

  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {
    leftFront = new TalonSRX(13);
    rightFront = new TalonSRX(2);
    leftRear = new TalonSRX(16);
    rightRear = new TalonSRX(14);
    rightFront.setInverted(true);
    rightRear.setInverted(true);
    leftDriveEncoder = new Encoder(0, 1);
    leftDriveEncoder.setReverseDirection(true);
    rightDriveEncoder = new Encoder(2, 3);
    leftDriveEncoder.setDistancePerPulse(0.0414);
    rightDriveEncoder.setDistancePerPulse(0.0414);
  }

  public void resetDriveEncoders() {
    leftDriveOffset = leftDriveEncoder.get();
    rightDriveOffset = rightDriveEncoder.get();
    leftDistanceOffset = leftDriveEncoder.getDistance();
    rightDistanceOffset = rightDriveEncoder.getDistance();
  }

  public int[] getDriveEncoder() {
    return new int[] { leftDriveEncoder.get() - leftDriveOffset, rightDriveEncoder.get() - rightDriveOffset };
  }

  public double[] getDriveDistance() {
    return new double[] { getLeftDistance(), getRightDistance() };
  }

  public double getLeftDistance() {
    return leftDriveEncoder.getDistance() - leftDistanceOffset;
  }

  public double getRightDistance() {
    return rightDriveEncoder.getDistance() - rightDistanceOffset;
  }

  public void setPower(double leftPower, double rightPower) {
    leftFront.set(ControlMode.PercentOutput, leftPower);
    leftRear.set(ControlMode.PercentOutput, leftPower);
    rightFront.set(ControlMode.PercentOutput, rightPower);
    rightRear.set(ControlMode.PercentOutput, rightPower);
  }

  public void arcadeDrive(double moveValue, double rotateValue, boolean squaredInputs) {
    double leftMotorSpeed = 0;
    double rightMotorSpeed = 0;

    moveValue *= yLimit;
    rotateValue *= xLimit;

    if (squaredInputs) {
      if (moveValue >= 0.0) {
        moveValue = moveValue * moveValue;
      } else {
        moveValue = -(moveValue * moveValue);
      }

      if (rotateValue >= 0.0) {
        rotateValue = rotateValue * rotateValue;
      } else {
        rotateValue = -(rotateValue * rotateValue);
      }

      if (moveValue > 0.0) {
        if (rotateValue > 0.0) {
          leftMotorSpeed = moveValue - rotateValue;
          rightMotorSpeed = Math.max(moveValue, rotateValue);
        } else {
          leftMotorSpeed = Math.max(moveValue, -rotateValue);
          rightMotorSpeed = moveValue + rotateValue;
        }
      } else {
        if (rotateValue > 0.0) {
          leftMotorSpeed = -Math.max(-moveValue, rotateValue);
          rightMotorSpeed = moveValue + rotateValue;
        } else {
          leftMotorSpeed = moveValue - rotateValue;
          rightMotorSpeed = -Math.max(-moveValue, -rotateValue);
        }
      }
      setPower(leftMotorSpeed, rightMotorSpeed);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
