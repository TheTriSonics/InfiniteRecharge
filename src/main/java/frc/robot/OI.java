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
        shiftHighButton.whenActive(shiftHighCommand);
        shiftLowButton.whenActive(shiftLowCommand);

        TrackTarget trackTarget = new TrackTarget();
        XboxTrigger trackingOnOff = new XboxTrigger(operator, XboxTrigger.Y);
        trackingOnOff.toggleWhenActive(trackTarget, true);

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


        
        /*
        XboxTrigger colorCount = new XboxTrigger(operator, XboxTrigger.DPADLEFT);
        colorCount.toggleWhenActive(new ColorChangeCount());
        */
        
    }

}


