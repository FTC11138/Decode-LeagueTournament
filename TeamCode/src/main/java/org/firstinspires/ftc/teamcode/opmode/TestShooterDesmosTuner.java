package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TEST Shooter Manual Tuner", group="Test")
public class TestShooterDesmosTuner extends LinearOpMode {

    // TODO: set these to your config names
    private static final String MOTOR_1_NAME = "shooterMotor1";
    private static final String MOTOR_2_NAME = "shooterMotor2";
    private static final String HOOD_SERVO_NAME = "adjHood";

    private static final double TICKS_PER_REV = 28.0;

    // step sizes
    private static final double HOOD_STEP = 0.005;   // servo pos
    private static final double TPS_STEP  = 25.0;    // ticks/sec

    private DcMotorEx shooter1, shooter2;
    private Servo hood;

    private double hoodPos = 0.0;   // 0..1
    private double tps = 0.0;       // ticks/sec target

    // edge detect
    private boolean pRB, pLB, pUp, pDown, pB;

    @Override
    public void runOpMode() {
        shooter1 = hardwareMap.get(DcMotorEx.class, MOTOR_1_NAME);
        shooter2 = hardwareMap.get(DcMotorEx.class, MOTOR_2_NAME);
        hood = hardwareMap.get(Servo.class, HOOD_SERVO_NAME);

        initMotor(shooter1);
        initMotor(shooter2);

        hood.setPosition(0.0);
        shooter1.setPower(0.0);
        shooter2.setPower(0.0);

        telemetry.addLine("Manual Shooter Tuner Ready");
        telemetry.addLine("RB/LB hood +/- | DpadUp/Down flywheel +/- | B stop");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean rb = gamepad1.right_bumper;
            boolean lb = gamepad1.left_bumper;
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            boolean b = gamepad1.b;

            // adjust hood
            if (rb && !pRB) hoodPos += HOOD_STEP;
            if (lb && !pLB) hoodPos -= HOOD_STEP;
            hoodPos = clamp01(hoodPos);

            // adjust flywheel target
            if (up && !pUp) tps += TPS_STEP;
            if (down && !pDown) tps -= TPS_STEP;
            if (tps < 0) tps = 0;

            // stop
            if (b && !pB) tps = 0;

            // apply
            hood.setPosition(hoodPos);

            if (tps <= 1e-6) {
                shooter1.setPower(0.0);
                shooter2.setPower(0.0);
            } else {
                shooter1.setVelocity(tps);
                shooter2.setVelocity(tps);
            }

            // measure
            double v1 = shooter1.getVelocity();
            double v2 = shooter2.getVelocity();
            double rpm1 = (v1 / TICKS_PER_REV) * 60.0;
            double rpm2 = (v2 / TICKS_PER_REV) * 60.0;

            // telemetry
            telemetry.addData("hoodPos (target)", "%.3f", hoodPos);
            telemetry.addData("flywheel tps (target)", "%.1f", tps);
            telemetry.addData("flywheel rpm (target)", "%.1f", (tps / TICKS_PER_REV) * 60.0);

            telemetry.addLine("--- measured ---");
            telemetry.addData("v1 (tps)", "%.1f", v1);
            telemetry.addData("v2 (tps)", "%.1f", v2);
            telemetry.addData("rpm1", "%.1f", rpm1);
            telemetry.addData("rpm2", "%.1f", rpm2);

            telemetry.update();

            // save prev
            pRB = rb; pLB = lb; pUp = up; pDown = down; pB = b;

            idle();
        }
    }

    private void initMotor(DcMotorEx motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double clamp01(double x) {
        if (x < 0) return 0;
        if (x > 1) return 1;
        return x;
    }
}