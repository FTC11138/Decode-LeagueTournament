package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;

public class SpindexerTestSubsystem extends RE_SubsystemBase {

    private final DcMotorEx spindexerMotor;
    private int targetPosition = 0;

    // ===== Constants =====
    private static final double TICKS_PER_REVOLUTION = 537.7; // adjust if needed
    private static final double TICKS_120_DEG = TICKS_PER_REVOLUTION / 3.0;
    private static final double MOVE_POWER = 0.2;

    public SpindexerTestSubsystem(HardwareMap hw, String motorName) {
        spindexerMotor = hw.get(DcMotorEx.class, motorName);

        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        targetPosition = 0;
        spindexerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Robot.getInstance().subsystems.add(this);
    }

    // ===== BASIC ROTATION FUNCTIONS =====

    /** Rotate 360 degrees clockwise */
    public void rotate360CW() {
        moveRelative(-TICKS_PER_REVOLUTION);
    }

    /** Rotate 360 degrees counter-clockwise */
    public void rotate360CCW() {
        moveRelative(TICKS_PER_REVOLUTION);
    }

    /** Rotate 120 degrees clockwise */
    public void rotate120CW() {
        moveRelative(-TICKS_120_DEG);
    }

    /** Rotate 120 degrees counter-clockwise */
    public void rotate120CCW() {
        moveRelative(TICKS_120_DEG);
    }

    /** Stop motor immediately */
    public void stop() {
        spindexerMotor.setPower(0);
    }

    /** Check if motor is currently moving */
    public boolean isMoving() {
        return spindexerMotor.isBusy();
    }

    /** Get current motor position */
    public int getCurrentPosition() {
        return spindexerMotor.getCurrentPosition();
    }

    /** Get target position */
    public int getTargetPosition() {
        return targetPosition;
    }

    // ===== INTERNAL HELPER =====
    private void moveRelative(double deltaTicks) {
        targetPosition += (int) deltaTicks;
        spindexerMotor.setTargetPosition(targetPosition);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexerMotor.setPower(MOVE_POWER);
    }

    @Override
    public void periodic() {
        // Nothing needed — RUN_TO_POSITION handles motion
    }
}