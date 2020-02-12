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

  double alpha = 0.45;
  double turnAlpha = 0.7;
  double lastTurn = 0;
  double turnAlpham1 = 1-turnAlpha;
  double alpham1 = 1 - alpha;
  double lastThrottle = 0;
  double lastSteering = 0;

  public ArcadeDriveCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double throttle = -Robot.oi.driver.getY(Hand.kLeft);
    double steering = -Robot.oi.driver.getX(Hand.kRight);
    double power = (alpha * throttle) + (alpham1 * lastThrottle);
    if (Math.abs(throttle) < Math.abs(lastThrottle)) power = throttle;
    double turn = (turnAlpha * steering) + turnAlpham1 * lastSteering;

    Robot.driveTrain.arcadeDrive(power, turn, true);
    lastThrottle = power;
    lastSteering = steering;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.driveTrain.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
