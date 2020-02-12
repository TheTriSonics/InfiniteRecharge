/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pneumatics extends SubsystemBase {
  public static final int SHIFT = 0;
  public static final int INTAKE = 1;
  public static final int CONTROL_WHEEL = 2;
  public static final int SHOOTER_HOOD = 3;
  public static final int HANGING_TILT = 4;
  public static final int HANGING_LOCK = 5;

  int numValves = 6;

  Solenoid[] solenoids = new Solenoid[numValves];
  boolean[] states = new boolean[numValves];

  public Pneumatics() {
    solenoids[SHIFT] = new Solenoid(Constants.SHIFT);
    solenoids[INTAKE] = new Solenoid(Constants.INTAKE);
    solenoids[CONTROL_WHEEL] = new Solenoid(Constants.CONTROL_WHEEL_EXTEND);
    solenoids[SHOOTER_HOOD] = new Solenoid(Constants.SHOOTER_HOOD);
    solenoids[HANGING_TILT] = new Solenoid(Constants.HANGING_TILT);
    solenoids[HANGING_LOCK] = new Solenoid(Constants.HANGING_LOCK);
    // for (int i = 0; i < solenoids.length; i++) states[i] = false;
  }

  public void setState(int valve, boolean state){
    solenoids[valve].set(state);
  }

  public void toggleState(int valve){
    solenoids[valve].set(!solenoids[valve].get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
