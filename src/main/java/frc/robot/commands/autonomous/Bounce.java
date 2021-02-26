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
public class Bounce extends SequentialCommandGroup {
  /**
   * Creates a new SimpleAutonomous.
   */

  public Bounce() {

    addCommands(
      new ExecuteProfile("bounce-1-profile.csv"),
      new SwitchDirection(),
      new ExecuteProfile("bounce-2-profile.csv"),
      new SwitchDirection(),
      new ExecuteProfile("bounce-3-profile.csv"),
      new SwitchDirection(),
      new ExecuteProfile("bounce-4-profile.csv")
    );
  }
}
