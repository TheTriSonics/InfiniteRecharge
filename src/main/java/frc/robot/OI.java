/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.*;
import frc.robot.utilities.XboxTrigger;

public class OI {
    public XboxController driver;
    public XboxController operator;

    public OI(){
        driver = new XboxController(0);
        operator = new XboxController(1);
        
        ShiftCommand shiftHighCommand = new ShiftCommand(false);
        ShiftCommand shiftLowCommand = new ShiftCommand(true);
        XboxTrigger shiftHighButton = new XboxTrigger(driver, XboxTrigger.RB);
        XboxTrigger shiftLowButton = new XboxTrigger(driver, XboxTrigger.LB);
        XboxTrigger goToGreen = new XboxTrigger(driver, XboxTrigger.A);
        XboxTrigger goToYellow = new XboxTrigger(driver, XboxTrigger.X);
        XboxTrigger goToOrange = new XboxTrigger(driver, XboxTrigger.Y);
        XboxTrigger goToRed = new XboxTrigger(driver, XboxTrigger.B);
        goToGreen.whenActive(new SetShooterData(0,18000));
        goToYellow.whenActive(new SetShooterData(800, 18000));
        goToOrange.whenActive(new SetShooterData(1050,18000));
        goToRed.whenActive(new SetShooterData(1100,18000));
        shiftHighButton.whenActive(shiftHighCommand);
        shiftLowButton.whenActive(shiftLowCommand);

        XboxTrigger switchDirection = new XboxTrigger(driver, XboxTrigger.DPADDOWN);
        switchDirection.whenActive(new SwitchDirection());

        /*
        XboxTrigger raiseCWRotation = new XboxTrigger(driver, XboxTrigger.DPADUP);
        XboxTrigger lowerCWRotation = new XboxTrigger(driver, XboxTrigger.DPADDOWN);
        raiseCWRotation.whenActive(new RaiseColorWheelRotation(true));
        lowerCWRotation.whenActive(new RaiseColorWheelRotation(false));

        XboxTrigger colorWheelRotate = new XboxTrigger(driver, XboxTrigger.DPADLEFT);
        XboxTrigger colorWheelColor = new XboxTrigger(driver, XboxTrigger.DPADRIGHT);
        raiseCWRotation.whenActive(new ColorWheelRotation());
        lowerCWRotation.whenActive(new ColorWheelChooseColor());
        */
        XboxTrigger hangingRelease = new XboxTrigger(driver, XboxTrigger.ENDGAME);
        hangingRelease.whenActive(new ReleaseHanging());

        ToggleTrackTarget toggleTrackTarget = new ToggleTrackTarget();
        XboxTrigger trackingOnOff = new XboxTrigger(operator, XboxTrigger.Y);
        trackingOnOff.toggleWhenActive(toggleTrackTarget, true);

        SpinUpShooterCommand spinUpShooterCommand = new SpinUpShooterCommand();
        XboxTrigger spinUpShooter = new XboxTrigger(operator, XboxTrigger.LB);
        spinUpShooter.whenActive(spinUpShooterCommand, true);

        ShooterOnCommand shooterOnCommand = new ShooterOnCommand();
        XboxTrigger shooterOn = new XboxTrigger(operator, XboxTrigger.RB);
        shooterOn.whenActive(shooterOnCommand, true);

        SetNothingCommand nothingCommand = new SetNothingCommand(true);
        XboxTrigger nothingButton = new XboxTrigger(operator, XboxTrigger.X);
        nothingButton.whenActive(nothingCommand, true);

        SetIntakeState setIntakeState = new SetIntakeState(true);
        XboxTrigger setIntakeButton = new XboxTrigger(operator, XboxTrigger.B);
        setIntakeButton.whenActive(setIntakeState);

        // ResetTurretToHome resetTurret = new ResetTurretToHome();
        // XboxTrigger resetTurretButton = new XboxTrigger(operator, XboxTrigger.A);
        // resetTurretButton.whenActive(resetTurret);

        XboxTrigger toggleManualShooting = new XboxTrigger(operator, XboxTrigger.DPADLEFT);
        toggleManualShooting.whenActive(new ToggleManualShooting());
        
        /*
        XboxTrigger colorCount = new XboxTrigger(operator, XboxTrigger.DPADLEFT);
        colorCount.toggleWhenActive(new ColorChangeCount());
        */
        
    }

}


