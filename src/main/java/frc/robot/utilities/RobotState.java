/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class RobotState {
    boolean shooterOn = false;
    boolean targetAligned = false;
    boolean intakeOn = false;
    boolean alignOn = false;

    public void toggleShooterOn(){
        shooterOn = !shooterOn;
        Robot.shooter.setShooterOn(shooterOn);
    }

    public void setShooterOn(boolean on){
        shooterOn = on;
    }

    public boolean isShooterOn(){
        return shooterOn;
    }

    public void setTargetAligned(boolean aligned){
        targetAligned = aligned;
    }

    public boolean isTargetAligned(){
        return targetAligned;
    }

    public void setIntakeOn(boolean on) {
        intakeOn = on;
    }

    public boolean isIntakeOn() {
        return intakeOn;
    }

    public void setAlignOn(boolean on) {
        alignOn = on;
    }

    public boolean isAligning() {
        return alignOn;
    }
}

