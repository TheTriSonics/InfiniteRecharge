/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand; 
  public static OI oi = new OI();
  public static RobotState robotState = new RobotState();
  
  public static InfiniteDriveTrain driveTrain;
  public static Turret turret;
  public static ShooterSubsystem shooter; 
  public static LimeLight limelight;
  public static NavX navx;
  public static ColorSensor colorSensor;
  public static Pneumatics pneumatics;
  public static ColorWheelRotateSubsystem colorWheelRotateSubsystem; 
  public static Position position;
  
  @Override
  public void robotInit() {
    driveTrain = new InfiniteDriveTrain();
    navx = new NavX();
    position = new Position();

    /*
    turret = new Turret();
    shooter = new ShooterSubsystem();
    limelight = new LimeLight();
    navx = new NavX();
    colorSensor = new ColorSensor();
    pneumatics = new Pneumatics();
    colorWheelRotateSubsystem = new ColorWheelRotateSubsystem();
    */
    
    driveTrain.setDefaultCommand(new ArcadeDriveCommand());
    /*
    turret.setDefaultCommand(new TurretCommand());
    limelight.setDefaultCommand(new LimeLightCommand());
    colorSensor.setDefaultCommand(new ColorSensorCommand());
    */
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = new ExecuteProfile("trenchtocenter-profile.csv");
    navx.resetGyro();
    position.resetPosition();
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("gyro", navx.getHeading());

  }

  double[] lastDriveEncoders;
  long lastTime;
  @Override
  public void teleopInit() {
    driveTrain.resetDriveEncoders();
    lastDriveEncoders = driveTrain.getDriveDistance();
    lastTime = System.currentTimeMillis();
    // System.out.println(Filesystem.getDeployDirectory());
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // navx.resetGyro();
    // position.resetPosition();
  }

  @Override
  public void teleopPeriodic() {
    // colorSensor.displayColor();
    // SmartDashboard.putNumber("gyro", navx.readGyro());
    double[] driveEncoders = driveTrain.getDriveDistance();
    long time = System.currentTimeMillis();
    double distance = VectorMath.avg(VectorMath.sub(driveEncoders, lastDriveEncoders));
    double velocity = distance/(time - lastTime) * 1000;
    lastTime = time;
    lastDriveEncoders = driveEncoders;
    // // System.out.println(velocity);
    SmartDashboard.putNumber("velocity", velocity);
    SmartDashboard.putNumber("left Drive", driveEncoders[0]);
    SmartDashboard.putNumber("right Drive", driveEncoders[1]);
    SmartDashboard.putNumber("gyro", navx.getHeading());
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
}
