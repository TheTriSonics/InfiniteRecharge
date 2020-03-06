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
  final double TURRET_OFFSET = 1500;
  final double TURRET_UPPER_LIMIT = 5150; // 4096+TURRET_OFFSET;//Check these values
  final double TURRET_LOWER_LIMIT = 1700;
  final double TURRET_HOME = 0;
  final double TURRET_TOLERANCE = 50;
  final double TURRET_MANUAL = 3380;
  final double HOOD_MANUAL = 500;
  final double HOOD_LOWER_LIMIT = 15;
  final double HOOD_UPPER_LIMIT = 1000;
  final double HOOD_RETRACT = 440;// HOOD_LOWER_LIMIT;
  final double HOOD_DEFAULT = HOOD_UPPER_LIMIT / 2;
  final double HOOD_TOLERANCE = 15;
  final double DEGREES_PER_ENCODER = 360.0/4096;
  final double kHood = 0.003; // .01;
  final double kTurret = 0.002; //0.0033;
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
    while(actualValue > 4096+TURRET_OFFSET) actualValue -= 4096;
    while(actualValue < TURRET_OFFSET) actualValue += 4096;
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
    double hoodPower = kHood * error;
    if (Math.abs(hoodPower) > 1) {
      if (hoodPower > 1) hoodPower = 1;
      else hoodPower = -1;
    }
    setRawHoodPower(kHood * error);
    hoodAligned = Math.abs(error) < HOOD_TOLERANCE;
    Robot.robotState.setHoodAligned(hoodAligned);
  }

  public void moveTurret() {
    double turretPosition = getTurretPosition();
    double error = turretTarget - turretPosition;
    //System.out.println(turretTarget + " " + turretPosition);
    double power = kTurret * error;
    if (Math.abs(power) > 1) {
      if (power > 1) power = 1;
      else power = -1;
    }
    // System.out.println(power);
    setSpinPower(kTurret * error);
    Robot.robotState.setTargetAligned(Math.abs(error) < TURRET_TOLERANCE);
    if(error <= TURRET_TOLERANCE || error >= -TURRET_TOLERANCE) turretAligned = true;
    else turretAligned = false;
  }

  double[] hoodCoefficients = new double[] {
    736.0559214774742, -34.596635146405546, 4.025394384508232,
    4.464876570594562, -0.9066887422459833, 0.05905653856800916,
    -0.001274507653020055};
    /*
    734.2017583806572, -29.113723283574636, 3.178476041921085,
    3.8276278763239837, -0.7133292453820426, 0.04055262922071082,
  -0.0006877119500577162};
  
    */

  public double determineHoodPositionFromCamera(double distance) {
    double sum = 0;
    double monomial = 1;
    for (int i = 0; i < hoodCoefficients.length; i++) {
      sum += hoodCoefficients[i] * monomial;
      monomial *= distance;
    }

    return Math.min(sum, 670);
    // return Math.max(sum, 875)-50;
    //return HOOD_DEFAULT;

    /*
    double slope = (Constants.farAngle - Constants.closeAngle) / (Constants.farDistance - Constants.closeDistance);
    double b = Constants.farAngle - (slope * Constants.farDistance);
    return (slope * distance) + b;
    */
  }

  @Override
  public void periodic() {

    // targetSeen = false; 
    // if (targetSeen == false) return; // remove

    if (Robot.robotState.isManualShooting()) {
      hoodTarget = HOOD_MANUAL;
      turretTarget = TURRET_MANUAL;
      moveHood();
      moveTurret();
      return;
    }
    boolean shooterOn = Robot.robotState.isShooterSpinning();
    if (shooterOn) {
      // Robot.pneumatics.setState(Pneumatics.SHOOTER_HOOD, true); 
    } 

    if(targetSeen == false) {
      if (shooterOn) hoodTarget = HOOD_DEFAULT;
      else {
        hoodTarget = HOOD_RETRACT;
        if (Math.abs(getHoodEncoder() - HOOD_RETRACT) < HOOD_TOLERANCE) {
          // System.out.println("retracting hood");
          // Robot.pneumatics.setState(Pneumatics.SHOOTER_HOOD, false);
        }
      }
      moveHood();
      if (turretTargetSet) moveTurret();
      else setSpinPower(0);
      return;
    }
    
    turretTargetSet = false;
    hoodTarget = determineHoodPositionFromCamera(targetLocation[1]);
    // System.out.println(hoodTarget + " " + getHoodEncoder());
    moveHood();

    turretTarget = getTurretPosition() + (targetLocation[0]+0)/DEGREES_PER_ENCODER;
    moveTurret();
    Robot.robotState.setTargetAligned(turretAligned && hoodAligned);
  }
}
