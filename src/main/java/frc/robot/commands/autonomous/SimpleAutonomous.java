/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SimpleAutonomous extends SequentialCommandGroup {
  /**
   * Creates a new SimpleAutonomous.
   */
  public SimpleAutonomous() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    // super();
    addCommands(
      // new DriveForDistance(0.6, 50, 0),
      // new RotateToHeading(0.8, -90),
      // new DriveForDistance(0.6, 50, -90)

      new ExecuteProfile("snatch-cells-profile.csv"),
      new SwitchDirection(),
      new ExecuteProfile("trenchtocenter-profile.csv"),
      new SwitchDirection(),
      new RotateToHeading(0.5, 22.5),
      new DriveForDistance(0.6, 55, 22.5)
    );
  }
}
