/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    boolean manualShooting = false;
    boolean endGame = false;
    TrackTarget trackTarget;
    
    public enum GSField{
        REDA, REDB, BLUEA, BLUEB
    };

    public GSField detectedField = null;

    public void createTrackTarget() {
        trackTarget = new TrackTarget();
    }

    public boolean isEndGame() {
        return endGame;
    }

    public void setEndGame(boolean b){
        endGame = b;
        if (endGame) Robot.hangingSubsystem.resetEncoders();
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

    public void toggleManualShooting() {
        manualShooting = !manualShooting;
        toggleSpinUpShooter();
    }

    public boolean isManualShooting() {
        return manualShooting;
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
        // return shooterOn && Robot.shooter.isShooterAtSpeed();// remove
        // return shooterOn && Robot.shooter.isShooterAtSpeed() && isTurretReady();
        return shooterOn;
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
        }
    }
    public void setAlignOn(boolean on) {
        alignOn = on;
    }

    public void setNothing(boolean b) {
        nothing = b;
        if (nothing) {
            intakeOn = false;
        }
    }

    public void setTargetAligned(boolean aligned){
        targetAligned = aligned;
    }

    public void toggleSpinUpShooter() {
        spinUpShooter = !spinUpShooter;
        if (spinUpShooter == false) shooterOn = false;
        if (tracking && !inAuton()) toggleTracking();
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

