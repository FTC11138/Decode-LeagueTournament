package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;

public class SpindexerTestSubsystem extends RE_SubsystemBase {

    private final DcMotorEx spindexerMotor;
    private int targetPosition = 0;

    private final AnalogInput ranger;

    // ===== Constants =====
    private static final double TICKS_PER_REVOLUTION = 537.7; // adjust if needed
    private static final double TICKS_120_DEG = TICKS_PER_REVOLUTION / 3.0;
//    private static final double MOVE_POWER = 0.4;

    public SpindexerTestSubsystem(HardwareMap hw, String motorName, String rangerSensor) {
        spindexerMotor = hw.get(DcMotorEx.class, motorName);

        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ranger = hw.get(AnalogInput.class, rangerSensor);

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

    public double getRangerVoltage() {
        return ((ranger.getVoltage() * 32.5)-2.6);
    }

    // ===== INTERNAL HELPER =====
    private void moveRelative(double deltaTicks) {
        targetPosition += (int) deltaTicks;
        spindexerMotor.setTargetPosition(targetPosition);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexerMotor.setPower(Constants.spindexerRotatePower);
    }

    @Override
    public void periodic() {
        // Nothing needed — RUN_TO_POSITION handles motion
    }
}

//    package org.firstinspires.ftc.teamcode.hardware.subsystems;
//
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.teamcode.hardware.Robot;
//import org.firstinspires.ftc.teamcode.util.Constants;
//import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;
//
//    public class SpindexerSubsystem extends RE_SubsystemBase {
//
//        private final DcMotorEx spindexerMotor;
//        private final DcMotorEx externalEncoder;
//        private final AnalogInput ranger;
//
//        public enum SpindexerState {
//            IDLE,
//            ROTATING,
//            HOLDING
//        }
//
//        public SpindexerState spindexerState;
//
//        private int targetPosition = 0;
//        private double previousRangerVoltage = 0;
//        private boolean hasInitialized = false;
//
//        // ===== Constants =====
//        private static final double TICKS_PER_REVOLUTION = 8192.0; // REV Through Bore Encoder
//        private static final double TICKS_120_DEG = TICKS_PER_REVOLUTION / 3.0;
//
//        // Distance threshold - adjust based on your setup
//        // Swyft Ranger outputs voltage: higher voltage = closer object
//        private static final double VOLTAGE_THRESHOLD = 0.5; // volts change to trigger rotation
//        private static final double VOLTAGE_DETECTION_MIN = 1.5; // minimum voltage to consider object detected
//
//        public SpindexerSubsystem(HardwareMap hw, String motorName, String encoderName, String rangerName) {
//            spindexerMotor = hw.get(DcMotorEx.class, motorName);
//            externalEncoder = hw.get(DcMotorEx.class, encoderName);
//            ranger = hw.get(AnalogInput.class, rangerName);
//
//            // Configure spindexer motor
//            spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//            // Configure external encoder
//            externalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            externalEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//            spindexerState = SpindexerState.IDLE;
//            targetPosition = 0;
//
//            Robot.getInstance().subsystems.add(this);
//        }
//
//        @Override
//        public void updateData() {
//            // Optional: Update telemetry data
//            // Robot.getInstance().data.spindexerState = spindexerState;
//        }
//
//        @Override
//        public void periodic() {
//            double currentRangerVoltage = ranger.getVoltage();
//            int currentPosition = externalEncoder.getCurrentPosition();
//
//            switch (spindexerState) {
//                case IDLE:
//                    // Initialize previous voltage on first run
//                    if (!hasInitialized) {
//                        previousRangerVoltage = currentRangerVoltage;
//                        hasInitialized = true;
//                    }
//
//                    // Check if distance has changed significantly (object detected)
//                    double voltageDelta = Math.abs(currentRangerVoltage - previousRangerVoltage);
//
//                    if (currentRangerVoltage > VOLTAGE_DETECTION_MIN && voltageDelta > VOLTAGE_THRESHOLD) {
//                        // Object detected - trigger rotation
//                        targetPosition = currentPosition + (int) TICKS_120_DEG;
//                        spindexerState = SpindexerState.ROTATING;
//                        previousRangerVoltage = currentRangerVoltage;
//                    }
//
//                    spindexerMotor.setPower(0);
//                    break;
//
//                case ROTATING:
//                    int positionError = targetPosition - currentPosition;
//
//                    if (Math.abs(positionError) < 20) { // Within tolerance
//                        spindexerState = SpindexerState.HOLDING;
//                    } else {
//                        // Constant power rotation
//                        spindexerMotor.setPower(Constants.spindexerRotatePower);
//                    }
//                    break;
//
//                case HOLDING:
//                    // Hold position briefly, then return to IDLE
//                    spindexerMotor.setPower(0);
//                    spindexerState = SpindexerState.IDLE;
//                    break;
//            }
//        }
//
//        // ===== MANUAL CONTROL METHODS =====
//
//        /** Manually rotate 120 degrees counter-clockwise */
//        public void manualRotate120CCW() {
//            if (spindexerState == SpindexerState.IDLE) {
//                targetPosition = externalEncoder.getCurrentPosition() + (int) TICKS_120_DEG;
//                spindexerState = SpindexerState.ROTATING;
//            }
//        }
//
//        /** Manually rotate 120 degrees clockwise */
//        public void manualRotate120CW() {
//            if (spindexerState == SpindexerState.IDLE) {
//                targetPosition = externalEncoder.getCurrentPosition() - (int) TICKS_120_DEG;
//                spindexerState = SpindexerState.ROTATING;
//            }
//        }
//
//        /** Force stop and return to IDLE */
//        public void forceStop() {
//            spindexerMotor.setPower(0);
//            spindexerState = SpindexerState.IDLE;
//        }
//
//        /** Reset encoder position */
//        public void resetEncoder() {
//            externalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            externalEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            targetPosition = 0;
//        }
//
//        // ===== GETTERS =====
//
//        public SpindexerState getState() {
//            return spindexerState;
//        }
//
//        public int getCurrentPosition() {
//            return externalEncoder.getCurrentPosition();
//        }
//
//        public int getTargetPosition() {
//            return targetPosition;
//        }
//
//        public double getRangerVoltage() {
//            return ranger.getVoltage();
//        }
//
//        public double getRangerDistance() {
//            // Swyft Ranger: Voltage to distance conversion
//            // Approximate: 2.6V = 10cm, scales roughly linearly
//            // Adjust formula based on your calibration
//            double voltage = ranger.getVoltage();
//            return (voltage / 2.6) * 10.0; // Returns distance in cm
//        }
//
//        public boolean isRotating() {
//            return spindexerState == SpindexerState.ROTATING;
//        }
//    }