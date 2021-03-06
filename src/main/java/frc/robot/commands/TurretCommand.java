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

public class TurretCommand extends CommandBase {
  /**
   * Creates a new TurretCommand.
   */
  public TurretCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Robot.robotState.inAuton() || Robot.robotState.isManualShooting()) return;
    double spinPower = Robot.oi.operator.getX(Hand.kRight);
    if (Math.abs(spinPower)<0.03)spinPower=0;
    else Robot.turret.setTurretTarget(Double.NaN);
    Robot.turret.setSpinPower(0.5*spinPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
