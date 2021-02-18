/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class DriveToPoint extends CommandBase {
  double targetX;
  double targetY;
  double speed;
  double distance;
  double lastDistance = 10000;
  double currentSpeed = 0.2;

  public DriveToPoint(double x, double y, double speed) {
    addRequirements(Robot.driveTrain);
    targetX = x;
    targetY = y;
    this.speed = speed;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    currentSpeed += 0.05;
    if (currentSpeed > speed) currentSpeed = speed;
    double[] position = Robot.position.getPosition();
    double changeInX = targetX - position[0];
    double changeInY = targetY - position[1];
    distance = Math.sqrt(changeInX * changeInX + changeInY * changeInY);
    double heading = Math.toDegrees(Math.atan2(changeInY, changeInX));
    double error = heading - Robot.navx.getHeading();
    while(error > 180) error -= 360;
    while(error < -180) error += 360;
    double correction = 0.02 * error;
    double ramp = 0.05 * distance;
    if(ramp > 1) ramp = 1;
    Robot.driveTrain.setPower((currentSpeed - correction) * ramp, (currentSpeed + correction) * ramp);
  }

  @Override
  public void end(boolean interrupted) {
    Robot.driveTrain.setPower(0, 0);
  }

  @Override
  public boolean isFinished() {
    boolean finished = distance > lastDistance;
    lastDistance = distance;
    return finished;  
  }
}
