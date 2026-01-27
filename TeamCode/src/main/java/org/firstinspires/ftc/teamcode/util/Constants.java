package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;

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

    public static int intakeUnstuckDelay = 100;
    public static int unstuckWait = 300;


    /* -------------------------------------------- SHOOT CONSTANTS -------------------------------------------- */

    public static double shootPower = -0.66;
    public static double readyPower = -1.0;
    public static double reverseStopPower = 1;
    public static double lowerShootPower = 0.71;
    public static double shootMultiplier = 1.0;

    public static double kP = 0.005; // to make response faster
    public static double kI = 0.000001; // for undershoot
    public static double kD = 0.0001; // don't change
    public static double kF = 32767 / 2800;


    public static double spindexer_kP = 0.005; // to make response faster
    public static double spindexer_kI = 0.000001; // for undershoot
    public static double spindexer_kD = 0.0001; // don't change

    public static double spindexerRotatePower = 0.5;

    public static double stuckCurrent = 2.5;

    public static int shootBetweenWait = 600;


    public static int sensorWait = 620; // WAS 750
    public static int shootSensorWait = 1250;

    public static int shootWait = 1800;

    public static double adjHoodMax = 0.02;
    public static double adjHoodMin = 0.48;

    public static double shootVelFar = -1900;

    public static double shootVelClose = -1425;

    public static double shootHoodFar = 0.38;
    public static double shootHoodClose = 0.38;


//    public static double ballDetectThreshold = 3.5;
//    public static int ballDetectWait = 100;

    // default must tune


    /* -------------------------------------------- TURRET CONSTANTS -------------------------------------------- */
    // Controller helper params
    public static double deadbandDeg = 0;
    public static double errAlpha = 0.35;

    public static double CENTER_KP = 0.008;   // lower than 0.02
    public static double CENTER_KD = 0.0001;

    // Safety rails
    public static double maxIntegral = 30.0;   // deg·s (anti-windup)
    public static double maxDeriv = 320.0;  // deg/s (D clamp)

    // CR servo output limits
    public static double maxPower = 1.0;
    public static double kS = 0.0;      //set to 0.03 is something is still messing up

    // PID gains mapping error->power (tune these)
//    public static double kP_v = 0.008;
//    public static double kI_v = 0.0005;
//    public static double kD_v = 0.0009;


    public static double kP_v = 0.015;
    public static double kI_v = 0;
    public static double kD_v = 0.001;//0.0010

    public static double kP_velo = 0.75;
    public static double kI_velo = 0.0;
    public static double kD_velo = 0.2;
    public static double kF_velo = 14.0;
}

