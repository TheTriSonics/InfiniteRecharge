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
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.*;
import frc.robot.utilities.VectorMath;

public class InfiniteDriveTrain extends SubsystemBase {
  TalonFX leftMaster, leftSlave1, leftSlave2, rightMaster, rightSlave1, rightSlave2;
  DutyCycleEncoder leftDriveEncoder, rightDriveEncoder;

  double leftDriveOffset = 0, rightDriveOffset = 0, leftDistanceOffset = 0, rightDistanceOffset = 0;
  double yLimit = 1;
  double xLimit = 1;
  boolean switched = false;

  public InfiniteDriveTrain() {
    leftDriveEncoder = new DutyCycleEncoder(Constants.DRIVE_TRAIN_LEFT);
    rightDriveEncoder = new DutyCycleEncoder(Constants.DRIVE_TRAIN_RIGHT);
    leftMaster = new TalonFX(Constants.LEFT_MASTER);
    leftSlave1 = new TalonFX(Constants.LEFT_SLAVE1);
    // leftSlave2 = new TalonFX(Constants.LEFT_SLAVE2);
    rightMaster = new TalonFX(Constants.RIGHT_MASTER);
    rightSlave1 = new TalonFX(Constants.RIGHT_SLAVE1);
    // rightSlave2 = new TalonFX(Constants.RIGHT_SLAVE2);

    leftDriveEncoder.setDistancePerRotation(-Constants.INCHES_PER_REV);
    rightDriveEncoder.setDistancePerRotation(Constants.INCHES_PER_REV);

    // leftMaster.setInverted(true);
    rightMaster.setInverted(true);
    rightSlave1.setInverted(true);
    // rightSlave2.setInverted(true);

    leftSlave1.follow(leftMaster);
    // leftSlave2.follow(leftMaster);4

    rightSlave1.follow(rightMaster);
    // rightSlave2.follow(rightMaster);

    leftMaster.setNeutralMode(NeutralMode.Coast);
    rightMaster.setNeutralMode(NeutralMode.Coast);
    leftSlave1.setNeutralMode(NeutralMode.Coast);
    // leftSlave2.setNeutralMode(NeutralMode.Coast);
    rightSlave1.setNeutralMode(NeutralMode.Coast);
    // rightSlave2.setNeutralMode(NeutralMode.Coast);

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
    Robot.position.resetLastDistances();
  }

  public double[] getDriveEncoder() {
    double left = leftDriveEncoder.get() - leftDriveOffset;
    double right = rightDriveEncoder.get() - rightDriveOffset;
    if (switched) {
      return new double[] {-right, -left};
    } 
    return new double[] {left, right};
  }

  public double[] getDriveDistance() {
    if (switched) {
      return new double[] {-getRightDistance() , -getLeftDistance()};
    }
    return new double[] { getLeftDistance(), getRightDistance() };
  }

  public double getDriveTotalDistance() {
    return VectorMath.avg(getDriveDistance());
  }

  public double getLeftDistance() {
    return leftDriveEncoder.getDistance() - leftDistanceOffset;
  }

  public double getRightDistance() {
    return rightDriveEncoder.getDistance() - rightDistanceOffset;
  }

  public void switchDirection() {
    TalonFX tmp = leftMaster;
    leftMaster = rightMaster;
    rightMaster = tmp;
    switched = !switched;
    resetDriveEncoders();
  }

  public boolean isSwitched() {
    return switched;
  }

  public void setPower(double leftPower, double rightPower){
    if (switched) {
      leftPower *= -1;
      rightPower *= -1;
    }
    double maxSpeed = 1;
    if (Math.abs(Robot.oi.driver.getTriggerAxis(Hand.kRight)) > 0.3 ||
        Robot.robotState.isEndGame()) maxSpeed = 0.4;
    leftMaster.set(ControlMode.PercentOutput, maxSpeed*leftPower);
    rightMaster.set(ControlMode.PercentOutput, maxSpeed*rightPower);
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
  }
}
