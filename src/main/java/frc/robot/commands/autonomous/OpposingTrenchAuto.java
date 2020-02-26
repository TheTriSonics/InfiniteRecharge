/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class OpposingTrenchAuto extends SequentialCommandGroup {
  /**
   * Creates a new OpposingTrenchAuto.
   */
  public OpposingTrenchAuto() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    ParallelCommandGroup initial = new ParallelCommandGroup(
      new SetIntakeState(true),
      new ExecuteProfile("snatch-cells-profile.csv")
    );

    addCommands(
      initial,
      new SwitchDirection(),
      new ExecuteProfile("trenchtocenter-profile.csv"),
      new SwitchDirection(),
      new RotateToHeading(0.5, 22.5),
      new DriveForDistance(0.6, 55, 22.5)
    );
  }
}
