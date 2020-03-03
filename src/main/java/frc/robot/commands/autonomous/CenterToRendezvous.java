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
public class CenterToRendezvous extends SequentialCommandGroup {
  /**
   * Creates a new CenterToRendezvous.
   */
  public CenterToRendezvous() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());

    ParallelCommandGroup initial = new ParallelCommandGroup(
      new SpinUpShooterCommand(),
      new ToggleTrackTarget(),
      new ShootForTime(4000)
    );

    addCommands(
      initial,
      new SpinUpShooterCommand(),
      new ToggleTrackTarget(),
      new SetIntakeState(true),
      new SetTurretTarget(2840),
      // new WaitForTime(5000),
      new ExecuteProfile("startcentertorsvp-profile.csv"),
      new SwitchDirection(),
      new DriveForDistance(0.4, 10, 112),
      new ToggleTrackTarget(),
      new SpinUpShooterCommand(),
      new ShootForTime(4000),
      new ToggleTrackTarget(),
      new SetIntakeState(false)
    );
    
  }
}
