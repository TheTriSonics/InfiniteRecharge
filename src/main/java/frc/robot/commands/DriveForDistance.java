/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveForDistance extends CommandBase {
  double stopDistance;
  double power;
  double heading;
  double currentPower;
  double distance;
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

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stopDistance = Robot.driveTrain.getLeftDistance() + distance;
    currentPower = 0.2;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPower = currentPower + 0.03;
    if(currentPower>power)currentPower = power;
    double remainingDistance = stopDistance - Robot.driveTrain.getLeftDistance();
    SmartDashboard.putNumber("remainingDistance", remainingDistance);
    if(remainingDistance<15)currentPower = power * remainingDistance/15;
    double angleError = heading - Robot.navx.getHeading();
    double correction = 0.02*angleError;
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
    return Robot.driveTrain.getLeftDistance()>stopDistance;
  }
}
