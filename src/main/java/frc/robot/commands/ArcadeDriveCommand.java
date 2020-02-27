/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ArcadeDriveCommand extends CommandBase {

  double alpha = 0.2;
  double turnAlpha = 0.7;
  double lastTurn = 0;
  double turnAlpham1 = 1-turnAlpha;
  double alpham1 = 1 - alpha;
  double lastThrottle = 0;
  double lastSteering = 0;

  public ArcadeDriveCommand() {
    addRequirements(Robot.driveTrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double throttle = -Robot.oi.driver.getY(Hand.kLeft);
    double steering = -Robot.oi.driver.getX(Hand.kRight);
    if(Math.abs(throttle) <= 0.05) throttle = 0;
    if(Math.abs(steering) <= 0.05) steering = 0;
    double power = (alpha * throttle) + (alpham1 * lastThrottle);
    if (Math.abs(throttle) < Math.abs(lastThrottle)) power = throttle;
    double turn = (turnAlpha * steering) + turnAlpham1 * lastSteering;

    Robot.driveTrain.arcadeDrive(power, turn, true);
    lastThrottle = power;
    lastSteering = steering;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.driveTrain.setPower(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
