/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.*;
import frc.robot.utilities.LEDMode;

public class LEDSubsystem extends SubsystemBase {
  private AddressableLED ledStrip;
  private AddressableLEDBuffer ledBuffer;

  private Timer timer;
  private double period;
  private LEDMode ledMode;
  private boolean usingRGB;
  private boolean cleared;
  private int[] rgb;
  private int[] hsv;

  /**
   * Creates a new LEDSubsystem.
   */
  public LEDSubsystem() {
    this.ledStrip = new AddressableLED(Constants.LED_STRIP);
    this.ledBuffer = new AddressableLEDBuffer(Constants.LED_STRIP_LENGTH);

    this.ledStrip.setLength(this.ledBuffer.getLength());
    this.ledStrip.setData(this.ledBuffer);
    this.ledStrip.start();

    // Initializing defaults
    this.timer = new Timer();
    this.period = 1.0;
    this.ledMode = LEDMode.SOLID;
    this.usingRGB = true;
    this.cleared = false;
    this.rgb = new int[3];
    this.hsv = new int[3];

    this.setPrimaryRGB(255, 0, 0);
    this.setAllRGB(255, 0, 0);
    this.enterMode(LEDMode.SOLID);
  }

  public void enterMode(LEDMode mode) {
    if (this.ledMode != mode) {
        this.timer.start();
    }
    this.ledMode = mode;
  }

  public void setPeriod(double period) {
      this.period = period;
  }

  public void setRGB(int index, int r, int g, int b) {
      this.ledBuffer.setRGB(index, r, g, b);
      this.ledStrip.setData(this.ledBuffer);
  }

  public void setHSV(int index, int h, int s, int v) {
    this.ledBuffer.setHSV(index, h, s, v);
    this.ledStrip.setData(this.ledBuffer);
  }

  public void setPrimaryRGB(int r, int g, int b) {
    this.rgb[0] = r;
    this.rgb[1] = g;
    this.rgb[2] = b;
  }

  public void setPrimaryHSV(int h, int s, int v) {
    this.hsv[0] = h;
    this.hsv[1] = s;
    this.hsv[2] = v;
  }

  public void setAllRGB(int r, int g, int b) {
      for (int i = 0; i < this.ledBuffer.getLength(); i++) {
          this.ledBuffer.setRGB(i, r, g, b);
      }
      this.ledStrip.setData(this.ledBuffer);
      this.usingRGB = true;
  }

  public void setAllHSV(int h, int s, int v) {
        for (int i = 0; i < this.ledBuffer.getLength(); i++) {
            this.ledBuffer.setHSV(i, h, s, v);
        }
        this.ledStrip.setData(this.ledBuffer);
        this.usingRGB = false;
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (this.ledMode == LEDMode.FLASH) {
        if (this.timer.get() >= this.period) {
            if (cleared) {
                this.setToCurrentConfig();
                this.cleared = false;
            } else {
                this.clear();
                this.cleared = true;
            }
            this.timer.start();
        }
    }
  }

  private void clear() {
      if (this.usingRGB) {
          this.setAllRGB(0, 0, 0);
      } else {
          this.setAllHSV(0, 0, 0);
      }
  }

  private void setToCurrentConfig() {
      if (this.usingRGB) {
        this.setAllRGB(this.rgb[0], this.rgb[1], this.rgb[2]);
      } else {
        this.setAllHSV(this.hsv[0], this.hsv[1], this.hsv[2]);
      }
  }
}
