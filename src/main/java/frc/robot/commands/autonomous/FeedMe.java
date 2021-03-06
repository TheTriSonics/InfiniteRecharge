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
public class FeedMe extends SequentialCommandGroup {
  /**
   * Creates a new FeedMe.
   */
  public FeedMe() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    ParallelCommandGroup initial = new ParallelCommandGroup(
      new SetIntakeState(true),
      new SpinUpShooterCommand(),
      new ToggleTrackTarget()
    );

    addCommands(
      initial,
      new ShootForTime(6000),
      new SpinUpShooterCommand(),
      new ToggleTrackTarget(),
      new RotateToHeading(0.5, 90),
      new DriveForDistance(0.7, 180, 90, 4000),
      new SwitchDirection(),
      new SetTurretTarget(3270),
      new SpinUpShooterCommand(),
      new DriveForDistance(0.5, 120, -90, 3000),
      new ToggleTrackTarget(),
      new ShootForTime(4000),
      new SpinUpShooterCommand(),
      new ToggleTrackTarget(),
      new SetIntakeState(false)
    );
  }
}
