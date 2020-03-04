/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.utilities.VectorMath;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveForDistance extends CommandBase {
  double stopDistance;
  double power;
  double heading;
  double currentPower;
  double distance;
  long stopTime;
  double speedRamp = 10.0;
  double kAngle = 0.02;
  double initialDistance;
  int timeout = 2000;
  /**
   * Creates a new DriveForDistance.
   */
  public DriveForDistance(double power, int distance, double heading) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
    this.power = power;
    this.distance = distance;
    this.heading = heading;
  }

  public DriveForDistance(double power, int distance, double heading, int timeout) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
    this.power = power;
    this.distance = distance;
    this.heading = heading;
    this.timeout = timeout;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stopTime = System.currentTimeMillis() + timeout;
    stopDistance = Robot.driveTrain.getDriveTotalDistance() + distance;
    currentPower = 0.2;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPower = currentPower + 0.03;
    if(currentPower>power)currentPower = power;
    double remainingDistance = stopDistance - Robot.driveTrain.getLeftDistance();
    SmartDashboard.putNumber("remainingDistance", remainingDistance);
    if (remainingDistance<speedRamp) currentPower = power * remainingDistance / speedRamp;
    double angleError = heading - Robot.navx.getHeading();
    angleError = VectorMath.normalizeAngle(angleError, 180);
    double correction = kAngle*angleError;
    Robot.driveTrain.setPower(currentPower - correction, currentPower + correction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.driveTrain.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Robot.driveTrain.getDriveTotalDistance() > stopDistance || System.currentTimeMillis() > stopTime;
  }
}
