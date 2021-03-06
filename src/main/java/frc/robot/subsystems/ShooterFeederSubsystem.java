/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.*;

public class ShooterFeederSubsystem extends SubsystemBase {
  // VictorSPX turretFeederVictor;
  
  public ShooterFeederSubsystem() {
    // turretFeederVictor = new VictorSPX(Constants.SHOOTER_FEEDER);
  }

  public void setPower(double power){
    // turretFeederVictor.set(ControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    // System.out.println(Robot.robotState.isShooterReady());
    if (Robot.robotState.isShooterReady()) {
      setPower(1);
      return;
    }
    setPower(0);
  }
}
