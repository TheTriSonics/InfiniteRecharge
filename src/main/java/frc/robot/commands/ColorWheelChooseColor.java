/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Pneumatics;

public class ColorWheelChooseColor extends CommandBase {
  /**
   * Creates a new ColorWheelChooseColor.
   */
  long startTime;
  String desiredColor;
  int lookForColor;
  boolean colorFound = false;
  public ColorWheelChooseColor() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.pneumatics.setState(Pneumatics.CONTROL_WHEEL, false);
    startTime = System.currentTimeMillis();
    desiredColor = DriverStation.getInstance().getGameSpecificMessage();
    switch(desiredColor.charAt(0)) {
      case 'B': {
        lookForColor = ColorSensor.YELLOW;
        break;
      }
      case 'G': {
        lookForColor = ColorSensor.YELLOW;
        break;
      }
      case 'R': {
        lookForColor = ColorSensor.YELLOW;
        break;
      }
      case 'Y': {
        lookForColor = ColorSensor.YELLOW;
        break;
      }
      default: {
        lookForColor = -1;
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (System.currentTimeMillis() < startTime + 500) return;
    Robot.colorWheelRotateSubsystem.setPower(0.4);
    int currentColor = Robot.colorSensor.getColor();
    if (currentColor == lookForColor) colorFound = true;
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
    return lookForColor == -1 || colorFound || Robot.pneumatics.getState(Pneumatics.CONTROL_WHEEL);
  }
}
