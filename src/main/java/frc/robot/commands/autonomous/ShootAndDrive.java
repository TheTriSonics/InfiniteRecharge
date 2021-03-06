/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;

public class ShootAndDrive extends SequentialCommandGroup {
  /**
   * Creates a new ShootAndDrive.
   */
  public ShootAndDrive() {
    ParallelCommandGroup initial = new ParallelCommandGroup(
      new SpinUpShooterCommand(),
      new ToggleTrackTarget()
    );
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      initial,
      new ShootForTime(4000),
      new SpinUpShooterCommand(),
      new ToggleTrackTarget(),
      new SwitchDirection(),
      new DriveForDistance(0.5, 60, 180)
    );
  }
  
}
