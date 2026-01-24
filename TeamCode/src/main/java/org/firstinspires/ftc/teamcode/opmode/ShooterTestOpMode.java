package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.Globals;

@TeleOp(name = "ShooterPureTuningOpMode")
public class ShooterTestOpMode extends OpMode {

    private final Robot robot = Robot.getInstance();
    private GamepadEx g1;

    private boolean enabled = false;

    // Hood tuning
    private double manualHoodPos = 0.5;
    private static final double HOOD_STEP = 0.01;

    // Shooter tuning (TPS / ticks per second)
    // NOTE: your flywheelSpeed() returns negative values, so start negative.
    private double manualShooterTPS = -1200;
    private static final double TPS_STEP = 25;

    // Edge detect latches
    private boolean lastRB = false, lastLB = false;
    private boolean lastDL = false, lastDR = false;
    private boolean lastA = false, lastB = false;

    private boolean shooterRunning = false;

    @Override
    public void init() {
        g1 = new GamepadEx(gamepad1);

        Globals.IS_AUTO = false;
        robot.initialize(hardwareMap, telemetry);

        // PURE tuning defaults
        robot.shooterSubsystem.updateShooterState(ShooterSubsystem.ShooterState.STOP);
        robot.shooterSubsystem.updateAdjHoodState(ShooterSubsystem.AdjHoodState.MANUAL);
        robot.shooterSubsystem.setManualHoodPos(manualHoodPos);

        // If your subsystem supports manual TPS override, keep this.
        // If not, comment this out and we can add it back after you paste your subsystem.
        robot.shooterSubsystem.setManualShooterTPS(manualShooterTPS);

        telemetry.addLine("Shooter PURE Tuning READY");
        telemetry.addLine("Press START to enable");
        telemetry.addLine("Hood: LB/RB step");
        telemetry.addLine("Shooter TPS: DPAD_LEFT/DPAD_RIGHT step");
        telemetry.addLine("Shooter: A=RUN, B=STOP");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Run robot loop (NO CommandScheduler here)
        robot.periodic();
        robot.updateData();
        robot.write();

        // Enable on START
        if (!enabled && g1.getButton(GamepadKeys.Button.START)) {
            enabled = true;
            gamepad1.rumble(500);
        }
        if (!enabled) return;

        // ✅ Re-assert manual hood every loop (prevents other code from overriding)
        robot.shooterSubsystem.updateAdjHoodState(ShooterSubsystem.AdjHoodState.MANUAL);

        // Hood step (edge-detected)
        boolean rb = g1.getButton(GamepadKeys.Button.RIGHT_BUMPER);
        boolean lb = g1.getButton(GamepadKeys.Button.LEFT_BUMPER);

        if (rb && !lastRB) manualHoodPos += HOOD_STEP;
        if (lb && !lastLB) manualHoodPos -= HOOD_STEP;

        lastRB = rb;
        lastLB = lb;

        manualHoodPos = Math.max(0.0, Math.min(1.0, manualHoodPos));
        robot.shooterSubsystem.setManualHoodPos(manualHoodPos);

        // Shooter TPS step (edge-detected)
        boolean dL = g1.getButton(GamepadKeys.Button.DPAD_LEFT);
        boolean dR = g1.getButton(GamepadKeys.Button.DPAD_RIGHT);

        if (dR && !lastDR) manualShooterTPS += TPS_STEP; // toward 0 if starting negative
        if (dL && !lastDL) manualShooterTPS -= TPS_STEP; // more negative

        lastDL = dL;
        lastDR = dR;

        // Apply manual shooter TPS (requires setManualShooterTPS in subsystem)
        robot.shooterSubsystem.setManualShooterTPS(manualShooterTPS);

        // Shooter run/stop
        boolean a = g1.getButton(GamepadKeys.Button.A);
        boolean b = g1.getButton(GamepadKeys.Button.B);

        if (a && !lastA) shooterRunning = true;
        if (b && !lastB) shooterRunning = false;

        lastA = a;
        lastB = b;

        // Use SHOOT state so periodic uses setVelocity().
        // Manual TPS override should win inside your subsystem.
        robot.shooterSubsystem.updateShooterState(
                shooterRunning ? ShooterSubsystem.ShooterState.SHOOT
                        : ShooterSubsystem.ShooterState.STOP
        );

        telemetry.addData("Enabled", enabled);
        telemetry.addData("AdjHoodState", robot.shooterSubsystem.adjHoodState);
        telemetry.addData("Manual Hood Pos", manualHoodPos);

        telemetry.addData("Shooter Running", shooterRunning);
        telemetry.addData("Manual Shooter TPS Target", manualShooterTPS);
        telemetry.addData("Actual TPS 1", robot.shooterSubsystem.getCurrentVelocity1());
        telemetry.addData("Actual TPS 2", robot.shooterSubsystem.getCurrentVelocity2());
        telemetry.update();
    }
}






