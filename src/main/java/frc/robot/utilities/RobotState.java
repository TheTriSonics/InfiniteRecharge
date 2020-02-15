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
    boolean spinUpShooter = false;
    boolean shooterOn = false;
    boolean targetAligned = false;
    boolean intakeOn = false;
    boolean alignOn = false;
    boolean nothing = true;
    boolean auton = false;

    public void setShooterOn(boolean on){
        shooterOn = on;
    }

    public boolean isShooterOn(){
        return shooterOn;
    }
    public boolean isTargetAligned(){
        return targetAligned;
    }

    public boolean inAuton() {
        return auton;
    }
    public boolean isIntakeOn() {
        return intakeOn;
    }
    public boolean isAligning() {
        return alignOn;
    }
    public boolean isNothing() {
        return nothing;
    }
    public boolean isShooterReady() {
        return shooterOn && Robot.shooter.isShooterAtSpeed();
    }

    public void setAuton(boolean b) {
        auton = b;
    }
    public void setIntakeOn(boolean on) {
        intakeOn = on;
        if (on) nothing = true;
    }
    public void setAlignOn(boolean on) {
        alignOn = on;
    }

    public void setNothing(boolean b) {
        nothing = b;
        if (nothing) intakeOn = false;
    }
    public void setTargetAligned(boolean aligned){
        targetAligned = aligned;
    }
    public void toggleSpinUpShooter() {
        spinUpShooter = !spinUpShooter;
        Robot.shooter.setShooterOn(spinUpShooter);
    }

    public void toggleShooterOn(){
        shooterOn = !shooterOn;
    }
}

