/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.VectorMath;

public class ColorSensor extends SubsystemBase {
  I2C.Port i2cPort = I2C.Port.kOnboard;
  ColorSensorV3 colorSensor; 
  Color detectedColor;
  int colorCode;

  public static final int YELLOW = 0;
  public static final int RED = 1;
  public static final int GREEN = 2;
  public static final int BLUE = 3;
  public static final String[] colors = {"Yellow", "Red", "Green", "Blue"};
  /**
   * Creates a new ColorSensor.
   */
  public ColorSensor() {
    colorSensor = new ColorSensorV3(i2cPort);
  }

  public void readColor() {
    detectedColor = colorSensor.getColor();
    double hue = convertToHue(detectedColor);
    if (hue > 300 || hue < 60) {
      colorCode = RED;
      return;
    }
    if (hue < 110) {
      colorCode = YELLOW;
      return;
    }
    if (hue < 150) {
      colorCode = GREEN;
      return;
    }
    colorCode = BLUE;
  }

  public double getHue() {
    return convertToHue(detectedColor);
  }

  public double getRed() {
    return detectedColor.red;
  }

  public double getGreen() {
    return detectedColor.green;
  }

  public double getBlue() {
    return detectedColor.blue;
  }

  public int getColor() {
    return colorCode;
  }

  public void displayColor(){
    if (detectedColor == null) {
      return;
    }
    /*
    SmartDashboard.putNumber("red", detectedColor.red);
    SmartDashboard.putNumber("green", detectedColor.green);
    SmartDashboard.putNumber("blue", detectedColor.blue);
    SmartDashboard.putNumber("hue", convertToHue(detectedColor));
    SmartDashboard.putString("color", colors[getColor()]);
    */
  }

  public double convertToHue(Color color) {
    double[] colorArray = new double[] {color.red, color.green, color.blue};
    double cmax = VectorMath.max(colorArray);
    double cmin = VectorMath.min(colorArray);
    double delta = cmax - cmin;
    if (cmax == color.red) 
      return 60*VectorMath.mod((color.green - color.red)/delta, 6);
      //return 60*(color.green - color.red)/delta;
    if (cmax == color.green) 
      return 60*((color.blue-color.red)/delta + 2);
    return 60*((color.red-color.green)/delta + 4);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
