package org.firstinspires.ftc.teamcode.subsystems.shooter;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class ShooterConstants {
    public static String leftShooterName = "leftShooterMotor";
    public static String rightShooterName = "rightShooterMotor";
    public static String pitchServoName = "shooterServo";

    public static double shooterEpsilon = 40;

    // Servo
    public static double highPose = 0.9;
    public static double middlePose = 0.6;
    public static double lowPose = 0.00;

    public static double near1Pose = 0.04;
    public static double near2Pose = 0.18;
    public static double near3Pose = 0.32;
    public static double near4Pose = 0.44;
    public static double near5Pose = 0.55;
    public static double far1Pose = 0.9;
    public static double far2Pose = 1;

    /**
     * In Ticks Per Second
     */
    // Motor
    public static double stopVelocity = 0;
    public static double maxVelocity = 1580;
    public static double fastVelocity = 1480;
    public static double slowLimitVelocity = 1260;
    public static double slowVelocity = 1200;
    public static double minVelocity = 980;
    public static double kP = 0.015;
    public static double kI = 0.09;
    public static double kD = 0.0;
    public static double kF = 0.0004;
}
