/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ConvayorSubsystem extends SubsystemBase {
  Talon con0, con1;

  public ConvayorSubsystem() {
    con0 = new Talon(1);
    con1 = new Talon(0);

    con0.setInverted(true);
    con1.setInverted(true);
  }

  public void setPower(double power){
    con0.set(power);
    con1.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setPower(Robot.oi.operator.getY(Hand.kLeft));
  }
}
