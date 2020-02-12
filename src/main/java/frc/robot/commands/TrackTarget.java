/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class TrackTarget extends CommandBase {
  /**
   * Creates a new TrackTarget.
   */
  public TrackTarget() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.turret);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Robot.limelight.isTargetSeen()==false){
      System.out.println("not seen");
      Robot.turret.setSpinPower(0);
      Robot.turret.setTiltPower(0);
      return;
    }
    double x = Robot.limelight.getX();
    double y = Robot.limelight.getY();
    //System.out.println(x + " " + y);
    double tiltPower = 0.04*y;
    double spinPower = 0.08*x;
    if(tiltPower>1)tiltPower = 1;
    if(tiltPower<-1)tiltPower = -1;
    if(spinPower>1)spinPower = 1;
    if(spinPower<-1)spinPower = -1;
    System.out.println("trackTarget" + spinPower);
    Robot.turret.setSpinPower(spinPower);
    Robot.turret.setTiltPower(tiltPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
