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
public class GalaticSearch extends SequentialCommandGroup {
  /**
   * Creates a new SimpleAutonomous.
   */
  public static enum Path {
    REDA, REDB, BLUEA, BLUEB
  }; 
  public Path currentPath = Path.REDA; 

  public GalaticSearch(Path path) {
    currentPath = path;
    String pathFileName = "";
    switch(currentPath) {
      case REDA:
        pathFileName = "gs-a_red-profile.csv";
        break;
      case REDB:
        pathFileName = "gs-b_red-profile.csv";
        break;
      case BLUEA:
        pathFileName = "gs-a_blue-profile.csv";
        break;
      case BLUEB:
        pathFileName = "gs-b_blue-profile.csv";
        break;
    }
    addCommands(
      new SetIntakeState(true),
      new ExecuteProfile(pathFileName)
    );
  }
}
