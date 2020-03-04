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
  public static PhotoEyes photoEyes;
  public static LEDSubsystem leds;

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
    photoEyes = new PhotoEyes();

    pneumatics = new Pneumatics();

    navx = new NavX();
    position = new Position();

    turret = new Turret();
    limelight = new LimeLight();
    leds = new LEDSubsystem();
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
    robotState.createTrackTarget();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    turret.setRawHoodPower(0);
    limelight.setLEDState(false);
  }

  @Override
  public void disabledPeriodic() {
    pneumatics.setState(Pneumatics.PHOTOEYE_RECEIVER, false);
    limelight.setLEDState(false);
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = new FeedMe();
    robotState.setAuton(true);
    pneumatics.setState(Pneumatics.SHIFT, true);
    navx.resetGyro();
    position.resetPosition();
    // turret.resetHoodEncoder();
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    position.updatePosition();
    controlLEDs();    

    // System.out.println(position.getPosition()[0]);
    // SmartDashboard.putNumber("gyro", navx.getHeading());

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
    // turret.resetHoodEncoder();
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
    
    /*
    SmartDashboard.putNumber("velocity", velocity);
    SmartDashboard.putNumber("left Drive", driveEncoders[0]);
    SmartDashboard.putNumber("right Drive", driveEncoders[1]);
    SmartDashboard.putNumber("gyro", navx.getHeading());
    */

    double hoodPower = oi.driver.getTriggerAxis(Hand.kRight);
    double leftTrigger = -oi.driver.getTriggerAxis(Hand.kLeft);
    if (Math.abs(leftTrigger) > 0.25) hoodPower = leftTrigger;
    
    this.controlLEDs();

    // position.updateGoalDistance(limelight.getY());
    // turret.setHoodPower(hoodPower);
    // System.out.println("shooter on = " + robotState.isShooterOn());
    
    SmartDashboard.putNumber("hood position", turret.getHoodEncoder());
    SmartDashboard.putNumber("turret position", turret.getTurretPosition());
    SmartDashboard.putNumber("Goal Distance", position.getDistance());
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  private void controlLEDs() {
    if (Robot.robotState.isShooterOn() && Robot.robotState.isShooterReady() && Robot.robotState.isTurretReady() && !Robot.ballDelivery.getTopPhotoeye() && !Robot.ballDelivery.getBottomPhotoeye()) {
      // No balls are present, send white led status.
      Robot.leds.setPrimaryRGB(255, 255, 255);
      Robot.leds.enterMode(LEDMode.SOLID);
    } else if (Robot.robotState.isShooterOn() && Robot.robotState.isShooterReady() && Robot.robotState.isTurretReady()) {
      Robot.leds.setPrimaryRGB(0, 255, 0);
      Robot.leds.enterMode(LEDMode.SOLID);
    } else if (Robot.robotState.isShooterOn() && (Robot.robotState.isShooterReady() || Robot.robotState.isTurretReady())) {
      Robot.leds.setPrimaryRGB(255, 0, 255);
      Robot.leds.enterMode(LEDMode.SOLID);
    } else if (Robot.robotState.isShooterOn()) {
      Robot.leds.setPrimaryRGB(255, 0, 255);
      Robot.leds.enterMode(LEDMode.FLASH);
    } else if (Robot.robotState.isIntakeOn()) {
      Robot.leds.setPrimaryRGB(255, 0, 0);
      Robot.leds.enterMode(LEDMode.FLASH);
    } else {
      Robot.leds.setPrimaryRGB(255, 0, 0);
      Robot.leds.enterMode(LEDMode.SOLID);
    }
  }
}
