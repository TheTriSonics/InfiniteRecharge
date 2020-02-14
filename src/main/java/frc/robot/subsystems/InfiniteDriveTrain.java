/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class InfiniteDriveTrain extends SubsystemBase {
  TalonFX leftMaster, leftSlave1, leftSlave2, rightMaster, rightSlave1, rightSlave2;
  DutyCycleEncoder leftDriveEncoder, rightDriveEncoder;

  double leftDriveOffset = 0, rightDriveOffset = 0, leftDistanceOffset = 0, rightDistanceOffset = 0;
  double yLimit = 1;
  double xLimit = 1;

  public InfiniteDriveTrain() {
    leftDriveEncoder = new DutyCycleEncoder(Constants.DRIVE_TRAIN_LEFT);
    rightDriveEncoder = new DutyCycleEncoder(Constants.DRIVE_TRAIN_RIGHT);
    leftMaster = new TalonFX(Constants.LEFT_MASTER);
    leftSlave1 = new TalonFX(Constants.LEFT_SLAVE1);
    // leftSlave2 = new TalonFX(Constants.LEFT_SLAVE2);
    rightMaster = new TalonFX(Constants.RIGHT_MASTER);
    rightSlave1 = new TalonFX(Constants.RIGHT_SLAVE1);
    // rightSlave2 = new TalonFX(Constants.RIGHT_SLAVE2);

    leftDriveEncoder.setDistancePerRotation(Constants.INCHES_PER_REV);
    rightDriveEncoder.setDistancePerRotation(-Constants.INCHES_PER_REV);

    // leftMaster.setInverted(true);
    rightMaster.setInverted(true);
    rightSlave1.setInverted(true);
    // rightSlave2.setInverted(true);

    leftSlave1.follow(leftMaster);
    // leftSlave2.follow(leftMaster);

    rightSlave1.follow(rightMaster);
    // rightSlave2.follow(rightMaster);

    leftMaster.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);
    leftSlave1.setNeutralMode(NeutralMode.Coast);
    rightSlave1.setNeutralMode(NeutralMode.Coast);

    leftMaster.configClosedloopRamp(1.5);
    leftSlave1.configClosedloopRamp(1.5);
    rightMaster.configClosedloopRamp(1.5);
    rightSlave1.configClosedloopRamp(1.5);
  }

  public void resetDriveEncoders() {
    leftDriveOffset = leftDriveEncoder.get();
    rightDriveOffset = rightDriveEncoder.get();
    leftDistanceOffset = leftDriveEncoder.getDistance();
    rightDistanceOffset = rightDriveEncoder.getDistance();
  }

  public double[] getDriveEncoder() {
    return new double[] { leftDriveEncoder.get() - leftDriveOffset, rightDriveEncoder.get() - rightDriveOffset};
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

  public void setPower(double leftPower, double rightPower){
    leftMaster.set(ControlMode.PercentOutput, leftPower);
    rightMaster.set(ControlMode.PercentOutput, rightPower);
  }

  public void arcadeDrive(double moveValue, double rotateValue, boolean squaredInputs) {
    double leftMotorSpeed = 0;
    double rightMotorSpeed = 0;

    moveValue *= yLimit;
    rotateValue *= xLimit;

    if(squaredInputs){
      if(moveValue >= 0.0){
        moveValue = moveValue * moveValue;
      } else {
        moveValue = -(moveValue * moveValue);
      }

      if(rotateValue >= 0.0) {
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
