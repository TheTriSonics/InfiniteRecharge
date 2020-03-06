/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Falcons
    public static final int LEFT_MASTER = 2;
    public static final int LEFT_SLAVE1 = 3;
    // public static final int LEFT_SLAVE2 = 3;
    public static final int RIGHT_MASTER = 5;
    public static final int RIGHT_SLAVE1 = 6;
    // public static final int RIGHT_SLAVE2 = 6;
    public static final int SHOOTER_MASTER = 7;
    public static final int SHOOTER_SLAVE = 8;

    // Victors
    public static final int BALL_DELIVERY1 = 6;
    public static final int BALL_DELIVERY2 = 5;
    public static final int HANGING_MOTOR1 = 4;
    public static final int HANGING_MOTOR2 = 3;
    public static final int THE_SINGULATOR = 1;
    public static final int INTAKE_MOTOR = 2;

    // CANCoder
    public static final int HOOD_ENCODER = 1;

    // Spark Max
    public static final int CONTROL_WHEEL = 1;

    // Talons
    public static final int TURRET_ROTATE = 9;
    public static final int LEFT_HANGING = 11;
    public static final int RIGHT_HANGING = 10;

    // Servo
    public static final int SHOOTER_HOOD_SERVO = 0;

    // Encoders
    public static final int DRIVE_TRAIN_LEFT = 6;
    public static final int DRIVE_TRAIN_RIGHT = 2;

    // Pneumatics
    public static final int SHIFT = 3;
    public static final int INTAKE = 2;
    public static final int CONTROL_WHEEL_EXTEND = 1;
    public static final int SHOOTER_HOOD = 0;
    public static final int HANGING_TILT = 4;
    public static final int HANGING_LOCK = 1;

    //AIO
    public static final int PHOTOEYE_TOP = 0;
    public static final int PHOTOEYE_BOTTOM = 1;

    //OTHER - PWM
    public static final int LED_STRIP = 9;

    // OTHER
    public static final double INCHES_PER_REV = (6.0 * Math.PI) * (100.0/97.0);
    public static final int LED_STRIP_LENGTH = 150;
    public static final double CAMERA_HEIGHT = 24.5; // 22.25 inches
    public static final double TARGET_HEIGHT = 98.25; // inches
    public static final double CAMERA_ANGLE = 19.0;

    // Temporary Points
    public static final double closeDistance = 118;
    public static final double farDistance = 239;

    public static final double closeAngle = 600;
    public static final double farAngle = 770;
}
