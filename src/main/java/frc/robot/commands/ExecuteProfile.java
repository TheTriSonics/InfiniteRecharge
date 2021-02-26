/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.utilities.CSVReader;
import frc.robot.utilities.VectorMath;

public class ExecuteProfile extends CommandBase implements Runnable {
  double[][] profile;
  int count;
  int profileLength;
  double vmax;
  Notifier notifier;
  boolean isNotifierRunning;
  /**
   * Creates a new ExecuteProfile.
   */
  public ExecuteProfile(String fileName) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
    CSVReader reader = new CSVReader(Filesystem.getDeployDirectory() + "/" + fileName); 
    profile = reader.parseCSV();
    vmax = reader.getVmax();
    profileLength = profile.length;
    count = 0;
    notifier = new Notifier(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.driveTrain.resetDriveEncoders();
    isNotifierRunning = true;
    notifier.startPeriodic(0.01);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.driveTrain.setPower(0, 0);
    notifier.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isNotifierRunning() == false;
  }

  public boolean isNotifierRunning() {
    return isNotifierRunning;
  }

  public void stopNotifier() {
    isNotifierRunning = false;
  }

  double kAngle = 0.03;
  double kDrive = 0.04;
  public void run() {
    double vmax = 2.2;
    if(count >= profileLength) {
      Robot.driveTrain.setPower(0,0);
      stopNotifier();
      return;
    }
    double[] data = profile[count];
    double leftPower = data[1] / vmax;
    double rightPower = data[3] / vmax;
    double[] driveDistance = Robot.driveTrain.getDriveDistance();
    double leftError = data[0] - driveDistance[0];
    double rightError = data[2] - driveDistance[1];
    leftPower += kDrive * leftError;
    rightPower += kDrive * rightError;
    double angleError = VectorMath.normalizeAngle(data[4] - Robot.navx.getHeading(), 180);
    SmartDashboard.putNumber("left error", leftError);
    SmartDashboard.putNumber("right error", rightError);
    SmartDashboard.putNumber("angle error", angleError);
    leftPower -= kAngle * angleError;
    rightPower += kAngle * angleError;
    Robot.driveTrain.setPower(leftPower, rightPower);
    count += 1;
    // System.out.println(count + " " + vmax + " " + leftPower + " " + rightPower);
  }
}
