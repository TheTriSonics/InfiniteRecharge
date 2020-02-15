/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.*;

public class SingulatorSubsystem extends SubsystemBase {
  VictorSPX singulatorVictor;

  public SingulatorSubsystem() {
    singulatorVictor = new VictorSPX(Constants.THE_SINGULATOR);
  }

  public void setPower(double power){
    singulatorVictor.set(ControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Robot.ballDelivery.getTopPhotoeye() && Robot.ballDelivery.getBottomPhotoeye()) {
      setPower(0);
      return;
    }
    if (Robot.robotState.isIntakeOn() || Robot.robotState.isShooterReady()) {
      setPower(1);
      return;
    }
    setPower(0);
  }
}
