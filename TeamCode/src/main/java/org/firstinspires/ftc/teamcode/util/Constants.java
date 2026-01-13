package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Constants {

    /* -------------------------------------------- DRIVE CONSTANTS -------------------------------------------- */


    public static double pathEndXTolerance = 1;
    public static double pathEndYTolerance = 1;
    public static double pathEndHeadingTolerance = Math.toRadians(2);

    public static boolean robotCentric = false;

    /* -------------------------------------------- CAMERA CONSTANTS -------------------------------------------- */
    //Pipeline: 0
    //Res: 1280X960 40FPS
    //Exposure: 252
    // Black Level Offset: 0
    // Sensor Gain: 15
    // Marker Size 101.6
    // Detector Downscale: 4
    // Quality Threshold: 2
    // Sort Mode: Largest

    /* -------------------------------------------- INTAKE CONSTANTS -------------------------------------------- */

    public static double intakeInPower = -1;
    public static double intakeOutPower = 1;

    public static double ballDetectThreshold = 0.3;
    public static int ballDetectWait = 170;

    public static int ballDetectWaitAuto = 160;

    /* -------------------------------------------- SHOOT CONSTANTS -------------------------------------------- */

    public static double shootPower = -0.7;
    public static double readyPower = -1.0;
    public static double reverseStopPower = 1;
    public static double lowerShootPower = 0.7;

    public static double kP = 0.005; // to make response faster
    public static double kI = 0.000001; // for undershoot
    public static double kD = 0.0001; // don't change
    public static double kF = 32767 / 2800;


    public static double spindexer_kP = 0.005; // to make response faster
    public static double spindexer_kI = 0.000001; // for undershoot
    public static double spindexer_kD = 0.0001; // don't change

    public static double spindexerRotatePower = 0.5;

    public static int sensorWait = 620; // WAS 750
    public static int shootSensorWait = 1250;

    public static int shootWait = 1800;

//    public static double ballDetectThreshold = 3.5;
//    public static int ballDetectWait = 100;

    // default must tune


    /* -------------------------------------------- TURRET CONSTANTS -------------------------------------------- */
    // Controller helper params
    public static double deadbandDeg = 0.30;
    public static double errAlpha = 0.35;

    // Safety rails
    public static double maxIntegral = 30.0;   // deg·s (anti-windup)
    public static double maxDeriv = 320.0;  // deg/s (D clamp)

    // CR servo output limits
    public static double maxPower = 1.0;
    public static double kS = 0.0;      //set to 0.03 is something is still messing up

    // PID gains mapping error->power (tune these)
    public static double kP_v = 0.020;
    public static double kI_v = 0.000;
    public static double kD_v = 0.0010;
}

