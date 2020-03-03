/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import frc.robot.Robot;
import frc.robot.commands.TrackTarget;
import frc.robot.utilities.LEDMode;

/**
 * Add your docs here.
 */
public class RobotState {
    boolean spinUpShooter = false;
    boolean shooterOn = false;
    boolean targetAligned = false;
    boolean hoodAligned = false;
    boolean intakeOn = false;
    boolean alignOn = false;
    boolean nothing = true;
    boolean auton = false;
    boolean tracking = false;
    TrackTarget trackTarget;

    public void createTrackTarget() {
        trackTarget = new TrackTarget();
    }

    public boolean isShooterSpinning(){
        return spinUpShooter;
    }
    public boolean isTargetAligned(){
        return targetAligned;
    }

    public void toggleTracking() {
        tracking = !tracking;
        if (tracking) trackTarget.schedule(true);
        else trackTarget.cancel();
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
        return shooterOn && Robot.shooter.isShooterAtSpeed() && isTurretReady();
    }

    public boolean isTurretReady() {
        return targetAligned && hoodAligned;
    }

    public void setHoodAligned(boolean aligned) {
        hoodAligned = aligned;
    }

    public void setAuton(boolean b) {
        auton = b;
    }
    public void setIntakeOn(boolean on) {
        intakeOn = on;
        if (on) {
            nothing = true;
            Robot.leds.setPrimaryRGB(255, 0, 0);
            Robot.leds.enterMode(LEDMode.FLASH);
        }
    }
    public void setAlignOn(boolean on) {
        alignOn = on;
    }

    public void setNothing(boolean b) {
        nothing = b;
        if (nothing) intakeOn = false;
        Robot.leds.setAllRGB(255, 0, 0);
        Robot.leds.enterMode(LEDMode.SOLID);
    }
    public void setTargetAligned(boolean aligned){
        targetAligned = aligned;
        Robot.leds.setPrimaryRGB(0, 255, 0);
        Robot.leds.enterMode(LEDMode.SOLID);
    }
    public void toggleSpinUpShooter() {
        spinUpShooter = !spinUpShooter;
        if (spinUpShooter == false) shooterOn = false;
        Robot.shooter.setShooterOn(spinUpShooter);
    }

    public void setShooterOff() {
        shooterOn = false;
    }

    public void setShooterOn() {
        shooterOn = true;
    }

    public void toggleShooterOn(){
        shooterOn = !shooterOn;
    }

    public boolean isShooterOn() {
        return shooterOn;
    }

}

