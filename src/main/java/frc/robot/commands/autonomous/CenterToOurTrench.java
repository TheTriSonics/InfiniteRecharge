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
import frc.robot.Robot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class CenterToOurTrench extends SequentialCommandGroup {
  /**
   * Creates a new CenterToOurTrench.
   */
  public CenterToOurTrench() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());

    ParallelCommandGroup initial = new ParallelCommandGroup(
      new SpinUpShooterCommand(),
      new ToggleTrackTarget()
    );

    ParallelCommandGroup driveThenIntake = new ParallelCommandGroup(
      new SequentialCommandGroup(new WaitForTime(50), new SetIntakeState(true)),
      new ExecuteProfile("trench-profile.csv")
    );

    addCommands(
      initial,
      new ShootForTime(4000),
      new SpinUpShooterCommand(),
      new ToggleTrackTarget(),
      driveThenIntake,
      //new ExecuteProfile("trench-profile.csv"),
      new SwitchDirection(),
      new SpinUpShooterCommand(),
      new ToggleTrackTarget(),
      new DriveForDistance(0.7, 100, 180),
      new SetIntakeState(false),
      new ShootForTime(4000),
      new SpinUpShooterCommand(),
      new ToggleTrackTarget()
      
      //new WaitForTime(3),
      //new ExecuteProfile("trench-profile.csv") // Not working?
    );
  }
}
