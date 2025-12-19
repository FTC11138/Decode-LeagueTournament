package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.hardware.subsystems.TurretOdometrySubsystem;

@TeleOp(name="Turret TrackPoint Test", group="Test")
public class TurretOdometryTuner extends OpMode {

    // --- CONFIG THESE NAMES TO MATCH YOUR RC CONFIG ---
    private static final String TURRET_SERVO_NAME = "turretServo";     // CRServo name
    private static final String TURRET_ENCODER_NAME = "turretEncoder"; // Motor port name used ONLY for encoder

    private Follower follower;
    private TurretOdometrySubsystem turret;

    // target point you can nudge with dpad
    private double targetX = 16.5;
    private double targetY = 131.0;

    // for button edge detection
    private boolean lastA = false;
    private boolean lastY = false;

    // for displaying debug
    private double lastServoCmd = 0.0;

    @Override
    public void init() {
        // Create your follower the same way your team normally does.
        // If you already have a Robot singleton that constructs it, use that.
        //
        // ---- IMPORTANT ----
        // Replace this with your team's normal follower creation.
        // Some teams do: follower = new Follower(hardwareMap);
        // Others: follower = new Follower(hardwareMap, ...);
        //
        this.follower = follower; // <-- change if your constructor differs

        turret = new TurretOdometrySubsystem(hardwareMap, TURRET_SERVO_NAME, TURRET_ENCODER_NAME, follower);
        turret.setTargetPoint(targetX, targetY);
        turret.setTurretState(TurretOdometrySubsystem.TurretState.MANUAL);

        telemetry.addLine("Turret TrackPoint Test ready.");
        telemetry.addLine("A = toggle TRACK_POINT, Y = zero encoder");
        telemetry.addLine("Left stick X = manual turret power");
        telemetry.addLine("Dpad = nudge target point");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update follower (MOST Pedro followers need an update call each loop)
        // If your Pedro version uses different method name, change it.
        follower.update();

        // --- Toggle tracking with A ---
        boolean a = gamepad1.a;
        if (a && !lastA) {
            if (turret.getTurretState() == TurretOdometrySubsystem.TurretState.MANUAL) {
                turret.setTurretState(TurretOdometrySubsystem.TurretState.TRACK_POINT);
            } else {
                turret.setTurretState(TurretOdometrySubsystem.TurretState.MANUAL);
            }
        }
        lastA = a;

        // --- Zero encoder with Y ---
        boolean y = gamepad1.y;
        if (y && !lastY) {
            turret.zeroTurretEncoder();
        }
        lastY = y;

        // --- Nudge target point with dpad (small increments) ---
        double step = gamepad1.left_bumper ? 0.25 : 1.0; // fine adjust with LB held

        if (gamepad1.dpad_up) targetY += step;
        if (gamepad1.dpad_down) targetY -= step;
        if (gamepad1.dpad_right) targetX += step;
        if (gamepad1.dpad_left) targetX -= step;

        turret.setTargetPoint(targetX, targetY);

        // --- Manual control when in MANUAL mode ---
        if (turret.getTurretState() == TurretOdometrySubsystem.TurretState.MANUAL) {
            // left stick x controls turret direction
            double manual = gamepad1.left_stick_x;

            // deadband
            if (Math.abs(manual) < 0.05) manual = 0.0;

            // scale to be safer while testing
            manual *= 0.5;

            lastServoCmd = manual;
            turret.setTurretPower(manual);
        }

        // --- Run subsystem periodic (PID runs here in TRACK_POINT) ---
        turret.periodic();

        // --- Telemetry / debug ---
        Pose pose = follower.getPose();
        double angleDeg = turret.getTurretAngleDeg();

        telemetry.addData("Mode", turret.getTurretState());
        telemetry.addData("Turret angle (deg)", "%.2f", angleDeg);
        telemetry.addData("Target (x,y)", "%.2f, %.2f", targetX, targetY);

        if (pose != null) {
            telemetry.addData("Pose (x,y,h)", "%.2f, %.2f, %.2f rad",
                    pose.getX(), pose.getY(), pose.getHeading());
        } else {
            telemetry.addLine("Pose: null (follower not providing pose)");
        }

        telemetry.addData("Manual servo cmd", "%.2f", lastServoCmd);
        telemetry.addLine("A toggle track | Y zero | Dpad moves target | LB = fine step");

        telemetry.update();
    }
}
