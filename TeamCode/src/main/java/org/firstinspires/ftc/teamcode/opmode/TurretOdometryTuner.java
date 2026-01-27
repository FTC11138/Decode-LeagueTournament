package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.util.Globals;
@Disabled
@TeleOp(name = "Turret Test", group = "Test")
public class TurretOdometryTuner extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Set alliance (IMPORTANT!)
        Globals.ALLIANCE = Globals.COLORS.BLUE; // Change to RED if needed
        Globals.IS_AUTO = false;

        // Initialize Robot (this creates all subsystems)
        Robot robot = Robot.getInstance();
        robot.initialize(hardwareMap, telemetry);

        telemetry.addLine("Turret Test OpMode");
        telemetry.addLine("================");
        telemetry.addLine("Controls:");
        telemetry.addLine("  Left Stick X: Manual turret control");
        telemetry.addLine("  A: Enable TRACK mode");
        telemetry.addLine("  B: Enable MANUAL mode");
        telemetry.addLine("  X: Reset encoder");
        telemetry.addLine("");
        telemetry.addLine("Press START when ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ========== CONTROLS ==========

            // Toggle between MANUAL and TRACK modes
            if (gamepad1.a) {
                robot.turretSubsystem.setTurretState(TurretSubsystem.TurretState.TRACK);
            }

            if (gamepad1.b) {
                robot.turretSubsystem.setTurretState(TurretSubsystem.TurretState.MANUAL);
            }

            // Manual control with left stick
            if (robot.turretSubsystem.getTurretState() == TurretSubsystem.TurretState.MANUAL) {
                double manualPower = -gamepad1.left_stick_x * 0.5; // Negative for intuitive control
                robot.turretSubsystem.setTurretPower(manualPower);
            }

            // Reset encoder
            if (gamepad1.x) {
//                robot.turretSubsystem.resetEncoder();
                sleep(200); // Debounce
            }

            // ========== UPDATE SUBSYSTEMS ==========
            robot.periodic();      // Updates all subsystems
            robot.updateData();    // Updates telemetry data

            // ========== TELEMETRY ==========
            telemetry.addData("Mode", robot.turretSubsystem.getTurretState());
            telemetry.addData("", "");

            telemetry.addData("Current Angle", "%.2f°", robot.turretSubsystem.getTurretAngleDeg());
            telemetry.addData("", "");
//
//            telemetry.addData("Camera Sees Basket?", robot.cameraSubsystem.hasBasket() ? "YES" : "NO");
//            if (robot.cameraSubsystem.hasBasket()) {
//                telemetry.addData("Basket Yaw Error", "%.2f°", robot.cameraSubsystem.getBasketYawDeg());
//                telemetry.addData("Distance", "%.2f m", robot.cameraSubsystem.getBasketDistanceM());
//                telemetry.addData("In Range?", robot.cameraSubsystem.getShootDistance());
//            }
//            telemetry.addData("", "");
//
//            telemetry.addData("Obelisk", robot.cameraSubsystem.getObelisk());
            telemetry.addData("", "");

            telemetry.addLine("Controls:");
            telemetry.addLine("  Left Stick X: Manual control");
            telemetry.addLine("  A: TRACK mode | B: MANUAL mode");
            telemetry.addLine("  X: Reset encoder");

            telemetry.update();
        }
    }
}