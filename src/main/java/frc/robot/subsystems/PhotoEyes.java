/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PhotoEyes extends SubsystemBase {
  /**
   * Creates a new PhotoEyes.
   */
  AnalogInput topPhotoEye, bottomPhotoEye;
  public PhotoEyes() {
    topPhotoEye = new AnalogInput(Constants.PHOTOEYE_TOP);
    bottomPhotoEye = new AnalogInput(Constants.PHOTOEYE_BOTTOM);
  }

  public double getTopVoltage() {
    return topPhotoEye.getVoltage();
  }

  public double getBottomVoltage() {
    return bottomPhotoEye.getVoltage();
  }

  public boolean getTopPhotoEye() {
    return topPhotoEye.getVoltage() > 0.8;
  }

  public boolean getBottomPhotoEye() {
    return bottomPhotoEye.getVoltage() > 0.8;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
