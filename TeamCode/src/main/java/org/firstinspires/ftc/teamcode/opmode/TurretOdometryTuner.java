package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.TurretOdometrySubsystem;
import org.firstinspires.ftc.teamcode.util.Globals;

@TeleOp(name = "Turret Test", group = "Test")
public class TurretOdometryTuner extends CommandOpMode {

    private final Robot robot = Robot.getInstance();
    private GamepadEx g1;

    // Target point you can nudge with dpad
    private double targetX = 16.5;
    private double targetY = 131.0;

    // Button state tracking
    private boolean lastA;
    private boolean lastY;

    @Override
    public void initialize() {
        g1 = new GamepadEx(gamepad1);

        Globals.IS_AUTO = false;

        // Initialize robot using your standard method
        robot.initialize(hardwareMap, telemetry);

        // Optional: Set a starting pose if needed
        // robot.follower.setStartingPose(new Pose(72, 72, Math.toRadians(0)));

        telemetry.addLine("Turret Test Ready!");
        telemetry.addLine("A = Toggle Auto Track");
        telemetry.addLine("Y = Zero Encoder");
        telemetry.addLine("Left Stick X = Manual Control");
        telemetry.addLine("Dpad = Nudge Target");
        telemetry.update();
    }

    @Override
    public void run() {
        // Update command scheduler
        CommandScheduler.getInstance().run();

        // Update robot systems
        robot.periodic();
        robot.updateData();
        robot.write();

        // Read buttons
        boolean a = g1.getButton(GamepadKeys.Button.A);
        boolean y = g1.getButton(GamepadKeys.Button.Y);

        // Toggle tracking with A button
        if (a && !lastA) {
            if (robot.turretOdometrySubsystem.getTurretState() == TurretOdometrySubsystem.TurretState.MANUAL) {
                robot.turretOdometrySubsystem.setTurretState(TurretOdometrySubsystem.TurretState.TRACK_POINT);
                gamepad1.rumble(200);
            } else {
                robot.turretOdometrySubsystem.setTurretState(TurretOdometrySubsystem.TurretState.MANUAL);
                gamepad1.rumble(100);
            }
        }

        // Zero encoder with Y button
        if (y && !lastY) {
            robot.turretOdometrySubsystem.zeroTurretEncoder();
            gamepad1.rumble(300);
        }

        // Update button states
        lastA = a;
        lastY = y;

        // Nudge target point with dpad
        double step = gamepad1.left_bumper ? 0.25 : 1.0; // Fine adjust with left bumper

        if (gamepad1.dpad_up) targetY += step;
        if (gamepad1.dpad_down) targetY -= step;
        if (gamepad1.dpad_right) targetX += step;
        if (gamepad1.dpad_left) targetX -= step;

        robot.turretOdometrySubsystem.setTargetPoint(targetX, targetY);

        // Manual control when in MANUAL mode
        if (robot.turretOdometrySubsystem.getTurretState() == TurretOdometrySubsystem.TurretState.MANUAL) {
            double manual = gamepad1.left_stick_x;

            // Deadband
            if (Math.abs(manual) < 0.05) {
                manual = 0.0;
            }

            // Scale for safety
            manual *= 0.5;

            robot.turretOdometrySubsystem.setTurretPower(manual);
        }

        // Telemetry
        Pose pose = robot.follower.getPose();
        double angleDeg = robot.turretOdometrySubsystem.getTurretAngleDeg();

        telemetry.addLine("=== TURRET ===");
        telemetry.addData("Mode", robot.turretOdometrySubsystem.getTurretState());
        telemetry.addData("Angle", "%.2f°", angleDeg);
        telemetry.addData("Target (x,y)", "%.2f, %.2f", targetX, targetY);
        telemetry.addData("RawTicks", robot.turretOdometrySubsystem.getRawTicks());

        telemetry.addLine();
        telemetry.addLine("=== ROBOT POSE ===");
        if (pose != null) {
            telemetry.addData("X", "%.2f", pose.getX());
            telemetry.addData("Y", "%.2f", pose.getY());
            telemetry.addData("Heading", "%.2f°", Math.toDegrees(pose.getHeading()));
        } else {
            telemetry.addLine("Pose: NULL");
        }

        telemetry.addLine();
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("A: Toggle Track | Y: Zero");
        telemetry.addLine("Left Stick X: Manual");
        telemetry.addLine("Dpad: Move Target | LB: Fine Adjust");

        telemetry.update();
    }
}