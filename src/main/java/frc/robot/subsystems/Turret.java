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
  Servo hood;
  TalonSRX spin;
  boolean targetSeen = false;
  DutyCycleEncoder hoodEncoder;
  SensorCollection sensors;
  double hoodTarget;
  double turretTarget;
  double[] targetLocation;

  final double UPPER_ENCODER_LIMIT = 4096;
  final double LOWER_ENCODER_LIMIT = -4096;
  final double HOOD_RETRACT = 0;
  final double HOOD_DEFAULT = 100;
  final double HOOD_TOLERANCE = 10;
  final double HOOD_LOWER_LIMIT = 0;
  final double HOOD_UPPER_LIMIT = 200;
  final double kHood = 0.1;
  final double kTurret = 0.1;
  private volatile int lastValue = Integer.MIN_VALUE;

  // private final AS5600EncoderPwm encoderPwm = new
  // AS5600EncoderPwm(spin.getSensorCollection()); //Absolute encoder for new robot turret

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
    // if (turretPosition > UPPER_ENCODER_LIMIT && power > 0) power = 0;
    // if (turretPosition < LOWER_ENCODER_LIMIT && power < 0) power = 0;
    spin.set(ControlMode.PercentOutput, power);
  }

  public void setTargetSeen(boolean targetSeen) {
    this.targetSeen = targetSeen;
  }
  
  public void setTargetLocation(double[] location) {
    targetLocation = location;
  }

  public double getHoodEncoder() {
    return hoodEncoder.get();
  }

  public void setHoodPower(double power) {
    double hoodPosition = hoodEncoder.getPositionOffset();
    if (hoodPosition < HOOD_LOWER_LIMIT && power < 0) power = 0;
    if (hoodPosition > HOOD_UPPER_LIMIT && power > 0) power = 0;
    hood.setSpeed(power);
  }

  public void moveHood() {
    double hoodPosition = hoodEncoder.get();
    double error = hoodTarget - hoodPosition;
    if (hoodPosition < HOOD_LOWER_LIMIT && error < 0) error = 0;
    if (hoodPosition > HOOD_UPPER_LIMIT && error > 0) error = 0;
    hood.setSpeed(kHood * error);
  }

  public double determineHoodPositionFromCamera(double y) {
    return HOOD_DEFAULT;
  }

  @Override
  public void periodic() {
    if (Robot.robotState.isShooterOn()) {
      Robot.pneumatics.setState(Pneumatics.SHOOTER_HOOD, true);
      if (targetSeen == false) hoodTarget = HOOD_DEFAULT;
    } else {
      hoodTarget = HOOD_RETRACT;
      moveHood();
      if (Math.abs(hoodEncoder.get() - HOOD_RETRACT) < HOOD_TOLERANCE) {
        Robot.pneumatics.setState(Pneumatics.SHOOTER_HOOD, false);
      }
    }
    if(targetSeen == false) return;
    
    hoodTarget = determineHoodPositionFromCamera(targetLocation[1]);
    moveHood();

    setSpinPower(kTurret*targetLocation[0]);
  }
}
