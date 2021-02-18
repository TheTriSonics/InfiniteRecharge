// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class SetShooterData extends CommandBase {
  double hoodTarget;
  int shooterSpeed;
  public SetShooterData(double target, int speed) {
    hoodTarget = target;
    shooterSpeed = speed;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    Robot.turret.setHoodTarget(hoodTarget);
    Robot.shooter.setShooterVelocity(shooterSpeed);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {}

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {}

}
