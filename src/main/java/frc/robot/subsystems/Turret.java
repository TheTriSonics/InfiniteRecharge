/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utilities.AS5600EncoderPwm;

public class Turret extends SubsystemBase {
  // TalonSRX spin;
  // Servo hoodTilt;
  TalonSRX spin;

  // private final AS5600EncoderPwm encoderPwm = new
  // AS5600EncoderPwm(spin.getSensorCollection()); //Absolute encoder for new robot turret

  public Turret() {
    spin = new TalonSRX(Constants.TURRET_ROTATE);
    // hoodTilt = new Servo(Constants.SHOOTER_HOOD_SERVO);
  }

  public void setSpinPower(double power) {
    spin.set(ControlMode.PercentOutput, power);
  }

  public void extendHood(boolean extend){
    Robot.pneumatics.setState(Pneumatics.SHOOTER_HOOD, extend);
  }

  @Override
  public void periodic() {
  }
}
