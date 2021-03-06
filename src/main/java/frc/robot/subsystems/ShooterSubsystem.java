/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ShooterSubsystem extends SubsystemBase {
  private static final double SHOOTER_TOLERANCE = 0.82;

  TalonFX master, follower;
  TalonFXSensorCollection sensors;

  private int shooterSpeed = 18000;

  public ShooterSubsystem() {
    master = new TalonFX(Constants.SHOOTER_MASTER);
    follower = new TalonFX(Constants.SHOOTER_SLAVE);
 
    sensors = master.getSensorCollection();

    master.setInverted(false);
    master.set(TalonFXControlMode.Velocity, 0);
    follower.setInverted(true);
    follower.set(TalonFXControlMode.Follower, Constants.SHOOTER_MASTER);;

    double kF = 1023.0/20600.0;
    double kP = 0.4; // 0.31; // Was 0.28
    master.config_kF(0, kF);
    master.config_kP(0, kP);
    master.config_kF(1, kF);
    master.config_kP(1, 0);
    // master.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    follower.config_kF(0, kF);
    follower.config_kP(0, kP);
    follower.config_kF(1, kF);
    follower.config_kP(1, 0);
  }

  public void setShooterVelocity(int velocity) {
    shooterSpeed = velocity;
    /*
    if (velocity >= 2000) {
      master.selectProfileSlot(0, 0);
    } else {
      master.selectProfileSlot(1, 0);
    }
    
    master.set(TalonFXControlMode.Velocity, velocity);
    // System.out.println(velocity);
    */
  }

  public void setShooterPower(double power) {
    master.set(ControlMode.PercentOutput, power);
  }
  
  public boolean isShooterAtSpeed(){
    // return sensors.getIntegratedSensorVelocity() > SHOOTER_TOLERANCE * shooterSpeed;
    return true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if(Robot.oi.driver.getBButton()) setShooterPower(1);
    SmartDashboard.putNumber("shooterSpeed", sensors.getIntegratedSensorVelocity());
    // if (Robot.robotState.isShooterSpinning()) master.set(TalonFXControlMode.Velocity, 18000);
    if (Robot.robotState.isShooterSpinning()) master.set(TalonFXControlMode.Velocity, shooterSpeed);
    else setShooterPower(0);
  }
}
