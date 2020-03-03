/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
 
public class Turret extends SubsystemBase {
  final double TURRET_UPPER_LIMIT = 4096;//Check these values
  final double TURRET_LOWER_LIMIT = -4096;
  final double TURRET_HOME = 0;
  final double TURRET_TOLERANCE = 30;
  final double HOOD_LOWER_LIMIT = 15;
  final double HOOD_UPPER_LIMIT = 1000;
  final double HOOD_RETRACT = HOOD_LOWER_LIMIT;
  final double HOOD_DEFAULT = HOOD_UPPER_LIMIT / 2;
  final double HOOD_TOLERANCE = 15;
  final double DEGREES_PER_ENCODER = 360.0/4096;
  final double kHood = .005;
  final double kTurret = 0.0033;
  private volatile int lastValue = Integer.MIN_VALUE;

  // Distance = 132, Angle = 800
  // Distance = 212, Angle = 600

  double hoodOffset = 0;
  Servo hood;
  TalonSRX spin;
  boolean targetSeen = false;
  CANCoder hoodEncoder;
  SensorCollection sensors;
  double hoodTarget = HOOD_LOWER_LIMIT;
  double turretTarget;
  boolean turretTargetSet = false;
  double[] targetLocation;
  boolean turretAligned = false;
  boolean hoodAligned = false;

  public Turret() {
    spin = new TalonSRX(Constants.TURRET_ROTATE);
    sensors = spin.getSensorCollection();
    hood = new Servo(Constants.SHOOTER_HOOD_SERVO);
    hoodEncoder = new CANCoder(Constants.HOOD_ENCODER);
  }

  public int getTurretPosition() {
    int raw = sensors.getPulseWidthRiseToFallUs();
    if (raw == 0) {
      int lastValue = this.lastValue;
      if (lastValue == Integer.MIN_VALUE) {
        return 0;
      }
      return lastValue;
    }
    int actualValue = Math.min(4096, raw - 128);
    lastValue = actualValue;
    return actualValue;
  }

  public void setSpinPower(double power) {
    int turretPosition = getTurretPosition();
    if (turretPosition > TURRET_UPPER_LIMIT && power > 0) power = 0;
    if (turretPosition < TURRET_LOWER_LIMIT && power < 0) power = 0;
    spin.set(ControlMode.PercentOutput, power);
  }

  public void resetTurret() {
    this.setTurretTarget(TURRET_HOME);
  }

  public void setTurretTarget(double target) {
    if (Double.isNaN(target)) {
      turretTargetSet = false;
      return;
    }
    turretTarget = target;
    turretTargetSet = true;
  }

  public void setTargetSeen(boolean targetSeen) {
    this.targetSeen = targetSeen;
  }
  
  public void setTargetLocation(double[] location) {
    targetLocation = location;
  }

  public void resetHoodEncoder() {
    hoodOffset = hoodEncoder.getPosition();
  }

  public double getHoodEncoder() {
    return hoodEncoder.getPosition() - hoodOffset;
  }

  public void setRawHoodPower(double power) {
    hood.set(1-0.5*(power+1));
  }

  public void setHoodPower(double power) {
    double hoodPosition = getHoodEncoder();
    if (hoodPosition < HOOD_LOWER_LIMIT && power < 0) power = 0;
    if (hoodPosition > HOOD_UPPER_LIMIT && power > 0) power = 0;
    setRawHoodPower(power);
  }

  public void moveHood() {
    double hoodPosition = getHoodEncoder();
    double error = hoodTarget - hoodPosition;
    if (hoodPosition < HOOD_LOWER_LIMIT && error < 0) error = 0;
    if (hoodPosition > HOOD_UPPER_LIMIT && error > 0) error = 0;
    // System.out.println("hood power = " + kHood*error + " " + hoodPosition + " " + hoodTarget);
    setRawHoodPower(kHood * error);
    if(error <= HOOD_TOLERANCE || error >= -HOOD_TOLERANCE) hoodAligned = true;
    else hoodAligned = false;
  }

  public void moveTurret() {
    double turretPosition = getTurretPosition();
    double error = turretTarget - turretPosition;
    setSpinPower(kTurret * error);
    if(error <= TURRET_TOLERANCE || error >= -TURRET_TOLERANCE) turretAligned = true;
    else turretAligned = false;
  }

  public double determineHoodPositionFromCamera(double distance) {
    double slope = (Constants.farAngle - Constants.closeAngle) / (Constants.farDistance - Constants.closeDistance);
    double b = Constants.farAngle - (slope * Constants.farDistance);
    return (slope * distance) + b;
  }

  @Override
  public void periodic() {
    boolean shooterOn = Robot.robotState.isShooterSpinning();
    if (shooterOn) {
      Robot.pneumatics.setState(Pneumatics.SHOOTER_HOOD, true); 
    } 

    if(targetSeen == false) {
      if (shooterOn) hoodTarget = HOOD_DEFAULT;
      else {
        hoodTarget = HOOD_RETRACT;
        if (Math.abs(getHoodEncoder() - HOOD_RETRACT) < HOOD_TOLERANCE) {
          Robot.pneumatics.setState(Pneumatics.SHOOTER_HOOD, false);
        }
      }
      moveHood();
      if (turretTargetSet) moveTurret();
      return;
    }
    
    turretTargetSet = false;
    hoodTarget = determineHoodPositionFromCamera(Robot.position.getDistance());
    moveHood();

        turretTarget = getTurretPosition() + targetLocation[0]/DEGREES_PER_ENCODER;
    moveTurret();
    Robot.robotState.setTargetAligned(turretAligned && hoodAligned);
  }
}
