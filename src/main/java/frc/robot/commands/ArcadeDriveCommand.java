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

  double alpha = 0.3;
  double turnAlpha = 0.5;
  double turnAlpham1 = 1-turnAlpha;
  double alpham1 = 1 - alpha;
  double lastYPower = 0;
  double lastTurn = 0;

  public ArcadeDriveCommand() {
    addRequirements(Robot.driveTrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // double throttle = -Robot.oi.driver.getY(Hand.kLeft);
    // double steering = -0.5*Robot.oi.driver.getX(Hand.kRight);
    // if(Math.abs(throttle) <= 0.05) throttle = 0;
    // if(Math.abs(steering) <= 0.05) steering = 0;
    // double power = (alpha * throttle) + (alpham1 * lastThrottle);
    // if (Math.abs(throttle) < Math.abs(lastThrottle)) power = throttle;
    // double turn = (turnAlpha * steering) + turnAlpham1 * lastSteering;

    // if (power < 0 && Math.abs(lastThrottle - power) > 0.25) {
    //   power = power * 0.7; // Attempt to do this slower going backwards...
    // }

    // Robot.driveTrain.arcadeDrive(power, turn, true);
    // lastThrottle = power;
    // lastSteering = steering;

    double yValues = -Robot.oi.driver.getY(Hand.kLeft);
    double xValues = -0.5 * Robot.oi.driver.getX(Hand.kRight);
    if (Math.abs(yValues) < 0.1) yValues = 0;
    if (Math.abs(xValues) < 0.1) xValues = 0;
    double power = (alpha * yValues) + (alpham1 * lastYPower);
    double turn = (turnAlpha * xValues) + turnAlpham1 * lastTurn;
    if (Math.abs(yValues) <= 0.15) {
    	power = (0.4 * yValues) + (0.65 * lastYPower);   
    	turn = xValues;
    }
    Robot.driveTrain.arcadeDrive(power, turn, true);
    lastYPower = power;
    lastTurn = turn;
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
