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
public class RightToOurTrench extends SequentialCommandGroup {
  /**
   * Creates a new RightToOurTrench.
   */
  public RightToOurTrench() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    ParallelCommandGroup initial = new ParallelCommandGroup(
      new SetIntakeState(true),
      new ExecuteProfile("straight2trench-profile.csv")
    );

    addCommands(
      new WaitForTime(50),
      initial
    );
  }
}
