package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.hardware.subsystems.TurretOdometrySubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;

/**
 * TeleOp to test the TurretOdometrySubsystem and tune PID.
 *
 * Controls:
 *  - gamepad1: drive robot (simple tank or mecanum, depending how you wire it)
 *  - gamepad2.a: set turret to TRACK_POINT
 *  - gamepad2.b: set turret to MANUAL
 *  - gamepad2.right_stick_x: manual turret power (in MANUAL mode)
 *
 *  PID tuning (gamepad2):
 *      dpad_up    : +kP
 *      dpad_down  : -kP
 *      dpad_right : +kD
 *      dpad_left  : -kD
 *      right_bump : +kI
 *      left_bump  : -kI
 *
 *      y          : +kS
 *      x          : -kS
 *
 *      start      : +maxPower
 *      back       : -maxPower
 */
@TeleOp(name = "Turret Odometry Tuner", group = "Test")
public class TurretOdometryTuner extends LinearOpMode {

    // --- Drivetrain (edit names to match your config) ---
    private DcMotorEx lf, rf, lb, rb;

    private Follower follower;
    private TurretOdometrySubsystem turret;

    // edge-detect booleans for tuning buttons
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastRB = false;
    private boolean lastLB = false;
    private boolean lastY = false;
    private boolean lastX = false;
    private boolean lastStart = false;
    private boolean lastBack = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // -----------------------
        // Drivetrain init (EDIT)
        // -----------------------
        // Change "lf","rf","lb","rb" to your motor names.
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse right side if needed
        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);

        // -----------------------
        // Follower init (EDIT)
        // -----------------------
        // TODO: replace this with your real PedroPathing follower init.
        // e.g. follower = Constants.createFollower(hardwareMap);
        follower = /* TODO: create your Follower here */ null;

        // -----------------------
        // Turret init
        // -----------------------
        // Change "turret" to your turret motor config name if different.
        turret = new TurretOdometrySubsystem(hardwareMap, "turret", follower);

        telemetry.addLine("Turret Odometry Tuner: init complete");
        telemetry.addLine("gamepad2 A=TRACK_POINT, B=MANUAL, RSX=manual power");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Update follower if you want live pose (recommended)
            if (follower != null) {
                follower.update();
            }

            // ---------------
            // Drive control
            // ---------------
            // Simple mecanum drive (edit as needed or strip out and push robot manually)
            double y = -gamepad1.left_stick_y; // forward
            double x = gamepad1.left_stick_x;  // strafe
            double r = gamepad1.right_stick_x; // rotate

            // Deadband
            double deadband = 0.05;
            if (Math.abs(y) < deadband) y = 0;
            if (Math.abs(x) < deadband) x = 0;
            if (Math.abs(r) < deadband) r = 0;

            double lfPower = y + x + r;
            double rfPower = y - x - r;
            double lbPower = y - x + r;
            double rbPower = y + x - r;

            // scale to max
            double max = Math.max(1.0,
                    Math.max(Math.abs(lfPower),
                            Math.max(Math.abs(rfPower),
                                    Math.max(Math.abs(lbPower), Math.abs(rbPower)))));

            lf.setPower(lfPower / max);
            rf.setPower(rfPower / max);
            lb.setPower(lbPower / max);
            rb.setPower(rbPower / max);

            // ---------------
            // Turret mode control
            // ---------------
            if (gamepad2.a) {
                turret.setTurretState(TurretOdometrySubsystem.TurretState.TRACK_POINT);
            } else if (gamepad2.b) {
                turret.setTurretState(TurretOdometrySubsystem.TurretState.MANUAL);
            }

            // Manual turret power when in MANUAL
            if (turret.getTurretState() == TurretOdometrySubsystem.TurretState.MANUAL) {
                double manualPower = gamepad2.right_stick_x;
                if (Math.abs(manualPower) < 0.05) manualPower = 0;
                turret.setTurretPower(manualPower);
            }

            // When in TRACK_POINT, turret subsystem's periodic / Robot loop
            // should be calling runTrackPointPID().
            // If not using Robot framework periodic, you can call:
            // turret.periodic();
            turret.periodic();

            // ---------------
            // PID TUNING (gamepad2)
            // ---------------

            // kP
            if (gamepad2.dpad_up && !lastDpadUp) {
                Constants.kP_v += 0.001;
            }
            if (gamepad2.dpad_down && !lastDpadDown) {
                Constants.kP_v -= 0.001;
            }

            // kD
            if (gamepad2.dpad_right && !lastDpadRight) {
                Constants.kD_v += 0.001;
            }
            if (gamepad2.dpad_left && !lastDpadLeft) {
                Constants.kD_v -= 0.001;
            }

            // kI
            if (gamepad2.right_bumper && !lastRB) {
                Constants.kI_v += 0.0001;
            }
            if (gamepad2.left_bumper && !lastLB) {
                Constants.kI_v -= 0.0001;
            }

            // kS
            if (gamepad2.y && !lastY) {
                Constants.kS += 0.001;
            }
            if (gamepad2.x && !lastX) {
                Constants.kS -= 0.001;
            }

            // maxPower
            if (gamepad2.start && !lastStart) {
                Constants.maxPower = Math.min(1.0, Constants.maxPower + 0.05);
            }
            if (gamepad2.back && !lastBack) {
                Constants.maxPower = Math.max(0.05, Constants.maxPower - 0.05);
            }

            // latch button states
            lastDpadUp = gamepad2.dpad_up;
            lastDpadDown = gamepad2.dpad_down;
            lastDpadLeft = gamepad2.dpad_left;
            lastDpadRight = gamepad2.dpad_right;
            lastRB = gamepad2.right_bumper;
            lastLB = gamepad2.left_bumper;
            lastY = gamepad2.y;
            lastX = gamepad2.x;
            lastStart = gamepad2.start;
            lastBack = gamepad2.back;

            // ---------------
            // Telemetry
            // ---------------
            Pose pose = (follower != null) ? follower.getPose() : null;
            telemetry.addLine("=== Turret Odometry Tuner ===");
            telemetry.addData("State", turret.getTurretState());
            telemetry.addData("TurretAngleDeg", turret.getTurretAngleDeg());

            if (pose != null) {
                telemetry.addData("Robot X", pose.getX());
                telemetry.addData("Robot Y", pose.getY());
                telemetry.addData("Heading (deg)", Math.toDegrees(pose.getHeading()));
            } else {
                telemetry.addLine("Pose: null (check follower init)");
            }

            telemetry.addLine("--- PID ---");
            telemetry.addData("kP_v", Constants.kP_v);
            telemetry.addData("kI_v", Constants.kI_v);
            telemetry.addData("kD_v", Constants.kD_v);
            telemetry.addData("kS", Constants.kS);
            telemetry.addData("maxPower", Constants.maxPower);

            telemetry.addLine("Controls:");
            telemetry.addLine("g2 A=TRACK_POINT, B=MANUAL, RSX=manual turret");
            telemetry.addLine("Tuning: g2 dpad: P, g2 bumpers: I, g2 dpad L/R: D, g2 X/Y: kS, g2 back/start: maxPower");
            telemetry.update();
        }
    }
}