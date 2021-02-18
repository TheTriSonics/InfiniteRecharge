/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class RotateWheelXTimesCommand extends CommandBase {
  int color;
  int count = 0;
  int lastColor;
  int numberOfRotations;
  double lastPosition;
  double position;
  double power = 0.25;

  public RotateWheelXTimesCommand(int numT) {
    addRequirements(Robot.colorWheelRotateSubsystem);
    numberOfRotations = numT * 8;
    lastPosition = Robot.colorWheelRotateSubsystem.getPosition();
    // count = 0;

  }

  @Override
  public void initialize() {
    lastColor = Robot.colorSensor.getColor();
  }

  @Override
  public void execute() {
    position = Robot.colorWheelRotateSubsystem.getPosition();
    Robot.colorWheelRotateSubsystem.setPower(power);
    color = Robot.colorSensor.getColor();
    if (color != lastColor) {
      count += 1;
      lastColor = color;
      SmartDashboard.putNumber("Position", position);
      SmartDashboard.putNumber("Last Position", lastPosition);
      SmartDashboard.putNumber("Change in Encoder", position - lastPosition);
      lastPosition = position;
    }
    
    SmartDashboard.putNumber("ColorCount", count);
    SmartDashboard.putNumber("Power", power);    
  }

  @Override
  public void end(boolean interrupted) {
    count = 0;
    Robot.colorWheelRotateSubsystem.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (count >= numberOfRotations);
  }
}
