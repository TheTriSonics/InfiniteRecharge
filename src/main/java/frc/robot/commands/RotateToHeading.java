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

public class RotateToHeading extends CommandBase {
  double heading;
  double power;
  double cutpoint;
  /**
   * Creates a new RotateToHeading.
   */
  public RotateToHeading(double power, double heading) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
    this.power = power;
    this.heading = heading;
    cutpoint = heading + 180;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = VectorMath.normalizeAngle(Robot.navx.getHeading(), cutpoint);
    double angleError = heading - angle;
    double powerFactor = 0.04 * angleError;
    if (powerFactor > 1) powerFactor = 1;
    if (powerFactor <-1) powerFactor = -1;
    Robot.driveTrain.setPower(-powerFactor*power, powerFactor*power);
    System.out.println(Robot.navx.getHeading());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.driveTrain.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(heading-Robot.navx.getHeading()) < 4;
  }
}
