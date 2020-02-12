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
public class PID {
    double Kp,Ki,Kd;
    double totalError = 0;
    double lastError = 0;
    double alpha = .8;
    double target;

    public PID(double Kp, double Ki, double Kd){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;

    }
    public void setTarget(double target){
        this.target = target;
        lastError = 0;
        totalError = 0;
    }

    public double getCorrection(double current){
        double error = target - current;
        double changeError = error - lastError;
        if (lastError * error <= 0) totalError = 0;
        else totalError = alpha * totalError + error;
        lastError = error;
        return (Kp*error+Ki*totalError+Kd*changeError);

    }
    public void setGains(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public double getTarget() {
        return target;
    }

}
