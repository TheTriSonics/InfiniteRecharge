/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Pneumatics;

public class ColorWheelRotation extends CommandBase {
  /**
   * Creates a new ColorWheelRotation.
   */
  long startTime;
  int currentColor = -1;
  int changes = 0;
  int desiredChanges = 50;
  public ColorWheelRotation() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.pneumatics.setState(Pneumatics.CONTROL_WHEEL, false);
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (System.currentTimeMillis() < startTime + 500) return;
    if (currentColor == -1) currentColor = Robot.colorSensor.getColor();
    Robot.colorWheelRotateSubsystem.setPower(0.5);
    int nextColor = Robot.colorSensor.getColor();
    if (nextColor != currentColor) {
      changes += 1;
      currentColor = nextColor;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.colorWheelRotateSubsystem.setPower(0);
    Robot.pneumatics.setState(Pneumatics.CONTROL_WHEEL, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return changes >= desiredChanges || Robot.pneumatics.getState(Pneumatics.CONTROL_WHEEL);
  }
}
