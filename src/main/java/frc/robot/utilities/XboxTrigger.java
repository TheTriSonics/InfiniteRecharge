/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

/**
 * Add your docs here.
 */

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.button.*;

public class XboxTrigger extends Trigger {
    public static final int A = 0;
    public static final int B = 1;
    public static final int X = 2;
    public static final int Y = 3;
    public static final int BACK = 4;
    public static final int RB = 5;
    public static final int LB = 6;
    public static final int START = 7;
    public static final int DPADUP = 8;
    public static final int DPADRIGHT = 9;
    public static final int DPADDOWN = 10;
    public static final int DPADLEFT = 11;
    public static final int DPADNE = 12;
    public static final int DPADSE = 13;
    public static final int DPADSW = 14;
    public static final int DPADNW = 15;
    public static final int LT = 16;
    public static final int RT = 17;
    public static final int ENDGAME = 18;

    XboxController xbox;
    int button;

    public XboxTrigger(XboxController xbox, int button) {
        this.xbox = xbox;
        this.button = button;
    }

    public boolean get() {
        switch (button) {
        case ENDGAME:
            return xbox.getBButton() && xbox.getYButton();
        case A:
            return xbox.getAButton();
        case B:
            return xbox.getBButton();
        case X:
            return xbox.getXButton();
        case Y:
            return xbox.getYButton();
        case BACK:
            return xbox.getBackButton();
        case RB:
            return xbox.getBumper(Hand.kRight);
        case LB:
            return xbox.getBumper(Hand.kLeft);
        case START:
            return xbox.getStartButton();
        case DPADUP:
            return xbox.getPOV(0) == 0;
        case DPADRIGHT:
            return xbox.getPOV(0) == 90;
        case DPADDOWN:
            return xbox.getPOV(0) == 180;
        case DPADLEFT:
            return xbox.getPOV(0) == 270;
        case DPADNE:
            return xbox.getPOV(0) == 45;
        case DPADSE:
            return xbox.getPOV(0) == 135;
        case DPADSW:
            return xbox.getPOV(0) == 225;
        case DPADNW:
            return xbox.getPOV(0) == 315;
        case LT:
            return Math.abs(xbox.getTriggerAxis(Hand.kLeft)) > 0.5;
        case RT:
            return Math.abs(xbox.getTriggerAxis(Hand.kRight)) > 0.5;
        }
        return false;
        
    }

}
