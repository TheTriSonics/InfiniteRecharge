/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utilities.VectorMath;

import java.lang.Math;

public class Position extends SubsystemBase {
  double positionX = 0;
  double positionY = 0;
  double goalDistance = 0;
  double[] lastDriveDistance;

  public Position() {
    // lastDriveDistance = Robot.infiniteDriveTrain.getDriveDistance();
    lastDriveDistance = Robot.driveTrain.getDriveDistance();
  }

  public double getDistance() {
    return goalDistance;
  }

  public double[] getPosition(){
    return new double[] {positionX, positionY};
  }

  public void resetLastDistances() {
    lastDriveDistance = Robot.driveTrain.getDriveDistance();
  }

  public void resetPosition() {
    positionX = 0;
    positionY = 0;
    // lastDriveDistance = Robot.infiniteDriveTrain.getDriveDistance();
    lastDriveDistance = Robot.driveTrain.getDriveDistance();
  }

  public void updatePosition() {
    double heading = Math.toRadians(Robot.navx.getHeading());
    double[] driveDistance = Robot.driveTrain.getDriveDistance();
    // double[] driveDistance = Robot.infiniteDriveTrain.getDriveDistance();
    double[] changeDriveDistance = VectorMath.sub(driveDistance, lastDriveDistance);
    double distance = VectorMath.avg(changeDriveDistance);
    positionX += distance * Math.cos(heading);
    positionY += distance * Math.sin(heading);
    lastDriveDistance = driveDistance;
  }

  /*
  public void updateGoalDistance(double goalY) {
    // goalDistance = (Constants.TARGET_HEIGHT - Constants.CAMERA_HEIGHT) / (Math.tan(Math.toRadians(Constants.CAMERA_ANGLE + goalY)));
  }
  */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updatePosition();
    // double[] position = getPosition();
    // SmartDashboard.putNumber("X", position[0]);
    // SmartDashboard.putNumber("Y", position[1]);
  }
}
