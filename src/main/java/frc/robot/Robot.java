/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand; 
  public static OI oi; 
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
  public static BallDeliverySubsystem ballDelivery;
  public static IntakeSubsystem intakeSubsystem;
  public static SingulatorSubsystem singulatorSubsystem;
  public static ShooterFeederSubsystem shooterFeederSubsystem;

  @Override
  public void robotInit() {
    Compressor compressor = new Compressor(0);
    compressor.setClosedLoopControl(true);

    driveTrain = new InfiniteDriveTrain();
    shooterFeederSubsystem = new ShooterFeederSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    singulatorSubsystem = new SingulatorSubsystem();
    ballDelivery = new BallDeliverySubsystem();
    shooter = new ShooterSubsystem();

    pneumatics = new Pneumatics();

    navx = new NavX();
    position = new Position();

    turret = new Turret();
    limelight = new LimeLight();
    /*
    colorSensor = new ColorSensor();
    colorWheelRotateSubsystem = new ColorWheelRotateSubsystem(); 
    */
    
    
    driveTrain.setDefaultCommand(new ArcadeDriveCommand());
    
    turret.setDefaultCommand(new TurretCommand());
    limelight.setDefaultCommand(new LimeLightCommand());
    /*
    colorSensor.setDefaultCommand(new ColorSensorCommand());
    */
    oi = new OI();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    turret.setRawHoodPower(0);
  }

  @Override
  public void disabledPeriodic() {
    
  }

  @Override
  public void autonomousInit() {
    robotState.setAuton(true);
    pneumatics.setState(Pneumatics.SHIFT, true);
    m_autonomousCommand = new OpposingTrenchAuto(); //ExecuteProfile("startcentertorsvp-profile.csv");  
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
    robotState.setAuton(false);
    if (driveTrain.isSwitched()) driveTrain.switchDirection();
    
    driveTrain.resetDriveEncoders();
    lastDriveEncoders = driveTrain.getDriveDistance();
    lastTime = System.currentTimeMillis();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    turret.resetHoodEncoder();
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

    /*
    double hoodPower = oi.driver.getTriggerAxis(Hand.kRight);
    double leftTrigger = -oi.driver.getTriggerAxis(Hand.kLeft);
    if (Math.abs(leftTrigger) > 0.2) hoodPower = leftTrigger;
    //hoodPower = 0.5*(hoodPower + 1);
    //System.out.println(hoodPower);
    //turret.setHoodPower(hoodPower);
    */
    SmartDashboard.putNumber("hood position", turret.getHoodEncoder());
    SmartDashboard.putNumber("turret position", turret.getTurretPosition());
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
}
