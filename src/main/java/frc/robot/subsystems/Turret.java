/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Turret extends SubsystemBase {
  final double TURRET_UPPER_LIMIT = 4096;
  final double TURRET_LOWER_LIMIT = -4096;
  final double HOOD_LOWER_LIMIT = 0;
  final double HOOD_UPPER_LIMIT = 3.1;
  final double HOOD_RETRACT = HOOD_LOWER_LIMIT;
  final double HOOD_DEFAULT = 2;
  final double HOOD_TOLERANCE = 0.5;
  final double DEGREES_PER_ENCODER = 360.0/4096;
  final double kHood = 1;
  final double kTurret = 0.003;
  private volatile int lastValue = Integer.MIN_VALUE;

  double hoodOffset = 0;
  Servo hood;
  TalonSRX spin;
  boolean targetSeen = false;
  DutyCycleEncoder hoodEncoder;
  SensorCollection sensors;
  double hoodTarget = HOOD_LOWER_LIMIT;
  double turretTarget;
  boolean turretTargetSet = false;
  double[] targetLocation;

  public Turret() {
    spin = new TalonSRX(Constants.TURRET_ROTATE);
    sensors = spin.getSensorCollection();
    hood = new Servo(Constants.SHOOTER_HOOD_SERVO);
    hoodEncoder = new DutyCycleEncoder(Constants.HOOD_ENCODER);
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
    hoodOffset = hoodEncoder.get();
  }

  public double getHoodEncoder() {
    return hoodEncoder.get() - hoodOffset;
  }

  public void setRawHoodPower(double power) {
    hood.set(1-0.5*(power+1));
  }

  public void setHoodPower(double power) {
    double hoodPosition = hoodEncoder.getPositionOffset();
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
  }

  public void moveTurret() {
    double turretPosition = getTurretPosition();
    double error = turretTarget - turretPosition;
    setSpinPower(kTurret * error);
  }

  public double determineHoodPositionFromCamera(double y) {
    return HOOD_DEFAULT;
  }

  @Override
  public void periodic() {
    boolean shooterOn = Robot.robotState.isShooterOn();
    if (shooterOn) {
      Robot.pneumatics.setState(Pneumatics.SHOOTER_HOOD, true); 
    } 

    if(targetSeen == false) {
      if (shooterOn) hoodTarget = HOOD_DEFAULT;
      else {
        hoodTarget = HOOD_RETRACT;
        if (Math.abs(hoodEncoder.get() - HOOD_RETRACT) < HOOD_TOLERANCE) {
          Robot.pneumatics.setState(Pneumatics.SHOOTER_HOOD, false);
        }
      }
      moveHood();
      if (turretTargetSet) moveTurret();
      return;
    }
    
    turretTargetSet = false;
    hoodTarget = determineHoodPositionFromCamera(targetLocation[1]);
    moveHood();

    turretTarget = getTurretPosition() + targetLocation[0]/DEGREES_PER_ENCODER;
    moveTurret();
  }
}
