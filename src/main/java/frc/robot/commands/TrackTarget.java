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

  public TrackTarget() {
    addRequirements(Robot.turret);
  }

  @Override
  public void initialize() {
    Robot.limelight.setLEDState(true);
  }

  @Override
  public void execute() {
    if(Robot.limelight.isTargetSeen()==false){
      Robot.turret.setTargetSeen(false);
      // System.out.println("not seen");
      Robot.turret.setSpinPower(0);
      return;
    }
    double x = Robot.limelight.getX();
    double y = Robot.limelight.getY();
    Robot.turret.setTargetSeen(true);
    Robot.turret.setTargetLocation(new double[]{x,y});
    
    /*
    double spinPower = 0.08*x;
    
    if(spinPower>1)spinPower = 1;
    if(spinPower<-1)spinPower = -1;
    System.out.println("trackTarget" + spinPower);
    Robot.turret.setSpinPower(spinPower);
    */
    
  }

  @Override
  public void end(boolean interrupted) {
    Robot.turret.setTargetSeen(false);
    Robot.limelight.setLEDState(false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
