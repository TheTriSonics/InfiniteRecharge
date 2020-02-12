/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakeSubsystem extends SubsystemBase {
  VictorSPX intakeVictor;

  public IntakeSubsystem() {
    intakeVictor = new VictorSPX(Constants.INTAKE_MOTOR);
  }

  public void setPower(double power) {
    intakeVictor.set(ControlMode.PercentOutput, power);
  }

  public void extendIntake(boolean extend){
    Robot.pneumatics.setState(Pneumatics.INTAKE, extend);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(Robot.robotState.isIntakeOn()){
      extendIntake(true);
      if(Robot.oi.operator.getTriggerAxis(Hand.kRight) > 0.05){
        setPower(Robot.oi.operator.getTriggerAxis(Hand.kRight));
      } else if(Robot.oi.operator.getTriggerAxis(Hand.kLeft) > 0.05){
        setPower(-Robot.oi.operator.getTriggerAxis(Hand.kLeft));
      } else {
        setPower(1);
      }
    }
  }
}
