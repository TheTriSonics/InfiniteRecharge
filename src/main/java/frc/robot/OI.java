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
        
        /*
        TrackTarget trackTarget = new TrackTarget();
        XboxTrigger trackingOnOff = new XboxTrigger(operator, XboxTrigger.Y);
        trackingOnOff.toggleWhenActive(trackTarget, true);
        */

        SpinUpShooterCommand spinUpShooterCommand = new SpinUpShooterCommand();
        XboxTrigger spinUpShooter = new XboxTrigger(operator, XboxTrigger.LB);
        spinUpShooter.toggleWhenActive(spinUpShooterCommand);

        ShooterOnCommand shooterOnCommand = new ShooterOnCommand();
        XboxTrigger shooterOn = new XboxTrigger(operator, XboxTrigger.RB);
        shooterOn.toggleWhenActive(shooterOnCommand, true);
        
        /*
        XboxTrigger colorCount = new XboxTrigger(operator, XboxTrigger.DPADLEFT);
        colorCount.toggleWhenActive(new ColorChangeCount());
        */
        
    }

}


