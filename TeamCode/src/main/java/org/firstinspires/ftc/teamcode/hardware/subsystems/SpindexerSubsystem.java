package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;

public class SpindexerSubsystem extends RE_SubsystemBase {

    private final DcMotorEx spindexerMotor;
    private final CameraSubsystem camera;


    private final ColorSensor slot1Inner;
    private final ColorSensor slot1Outer;
    private final ColorSensor slot2Inner;
    private final ColorSensor slot2Outer;
    private final ColorSensor slot3Inner;
    private final ColorSensor slot3Outer;

    private static final double TICKS_PER_REVOLUTION = 1440.0;
    private static final double SLOT_1_POSITION = 0.0; // Change Pos based on the position actaully
    private static final double SLOT_2_POSITION = TICKS_PER_REVOLUTION / 3.0;
    private static final double SLOT_3_POSITION = (2.0 * TICKS_PER_REVOLUTION) / 3.0;

    private static final double POSITION_TOLERANCE = 50.0; //Prevent nasty overshoot and undershoot
    private static final double ROTATION_POWER = 0.5;
    private static final double SHOOT_POWER = 0.8;

    private static final int GREEN_HUE_MIN = 80;
    private static final int GREEN_HUE_MAX = 160;
    private static final int MIN_SATURATION = 100;
    private static final int MIN_VALUE = 100;

    public enum SpindexerState {
        IDLE,
        SORTING,
        POSITIONING,
        SHOOTING,
        CW,
        CCW
    }

    private SpindexerState currentState;
    private int targetSlot; // 1, 2, or 3
    private double targetPosition;

    public SpindexerSubsystem(HardwareMap hw, String motorName,
                              String slot1InnerName, String slot1OuterName,
                              String slot2InnerName, String slot2OuterName,
                              String slot3InnerName, String slot3OuterName,
                              CameraSubsystem camera) {

        this.spindexerMotor = hw.get(DcMotorEx.class, motorName);
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.camera = camera;

        this.slot1Inner = hw.get(ColorSensor.class, slot1InnerName);
        this.slot1Outer = hw.get(ColorSensor.class, slot1OuterName);
        this.slot2Inner = hw.get(ColorSensor.class, slot2InnerName);
        this.slot2Outer = hw.get(ColorSensor.class, slot2OuterName);
        this.slot3Inner = hw.get(ColorSensor.class, slot3InnerName);
        this.slot3Outer = hw.get(ColorSensor.class, slot3OuterName);

        currentState = SpindexerState.IDLE;
        targetSlot = 1;
        targetPosition = SLOT_1_POSITION;

        Robot.getInstance().subsystems.add(this);
    }


    public void setState(SpindexerState newState) {
        if (currentState != newState) {
            currentState = newState;
            onStateChange(newState);
        }
    }

    public SpindexerState getState() {
        return currentState;
    }

    public void startSorting() {
        setState(SpindexerState.SORTING);
    }

    public void moveToSlot(int slot) {
        if (slot < 1 || slot > 3) {
            return; // Invalid slot
        }

        targetSlot = slot;
        updateTargetPosition();
        setState(SpindexerState.POSITIONING);
    }

    private int getGreenSlotFromObelisk(CameraSubsystem.Obelisk obelisk) {
        switch (obelisk) {
            case GPP:
                return 1; // Green in slot 1
            case PGP:
                return 2; // Green in slot 2
            case PPG:
                return 3; // Green in slot 3
            case PPP:
            default:
                return 0; // No green or invalid pattern
        }
    }

    public CameraSubsystem.Obelisk getObeliskForCurrentSlot() {
        switch (targetSlot) {
            case 1:
                return CameraSubsystem.Obelisk.GPP;
            case 2:
                return CameraSubsystem.Obelisk.PGP;
            case 3:
                return CameraSubsystem.Obelisk.PPG;
            default:
                return CameraSubsystem.Obelisk.PPP;
        }
    }

    public int getTargetSlot() {
        return targetSlot;
    }

    public void startShooting() {
        setState(SpindexerState.SHOOTING);
    }

    public void stopShooting() {
        setState(SpindexerState.IDLE);
    }

    public void stop() {
        setState(SpindexerState.IDLE);
    }

    public void rotateCW() {
        setState(SpindexerState.CW);
    }

    public void rotateCCW() {
        setState(SpindexerState.CCW);
    }

    public void resetEncoder() {
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        targetSlot = 1;
        targetPosition = SLOT_1_POSITION;
    }

    public double getCurrentPosition() {
        return spindexerMotor.getCurrentPosition();
    }

    public boolean isAtTarget() {
        return Math.abs(getCurrentPosition() - targetPosition) < POSITION_TOLERANCE;
    }


    public boolean isSlotGreen(int slot) {
        switch (slot) {
            case 1:
                return isGreen(slot1Inner) || isGreen(slot1Outer);
            case 2:
                return isGreen(slot2Inner) || isGreen(slot2Outer);
            case 3:
                return isGreen(slot3Inner) || isGreen(slot3Outer);
            default:
                return false;
        }
    }

    public String getSlotColorName(int slot) {
        if (isSlotGreen(slot)) {
            return "GREEN";
        } else {
            return "PURPLE";
        }
    }

    public boolean verifyGreenAtTarget() {
        return isSlotGreen(targetSlot);
    }


    @Override
    public void updateData() {
        // Update robot data for telemetry (uncomment and adjust as needed)
        // Robot.getInstance().data.spindexerState = currentState.name();
        // Robot.getInstance().data.spindexerSlot = targetSlot;
        // Robot.getInstance().data.spindexerPosition = getCurrentPosition();
        // Robot.getInstance().data.spindexerAtTarget = isAtTarget();
    }

    @Override
    public void periodic() {
        switch (currentState) {
            case IDLE:
                handleIdleState();
                break;

            case SORTING:
                handleSortingState();
                break;

            case POSITIONING:
                handlePositioningState();
                break;

            case SHOOTING:
                handleShootingState();
                break;

            case CW:
                handleCWState();
                break;

            case CCW:
                handleCCWState();
                break;
        }
    }


    private void handleIdleState() {
        spindexerMotor.setPower(0.0);
    }

    private void handleSortingState() {
        CameraSubsystem.Obelisk obelisk = camera.getObelisk();

        if (obelisk == null || obelisk == CameraSubsystem.Obelisk.PPP) {
            // No pattern found. Never will happen but just in case
            spindexerMotor.setPower(0.0);
        } else {

            int greenSlot = getGreenSlotFromObelisk(obelisk);

            if (greenSlot > 0) {
                targetSlot = greenSlot;
                updateTargetPosition();
                setState(SpindexerState.POSITIONING);
            } else {
                spindexerMotor.setPower(0.0);
            }
        }
    }

    private void handlePositioningState() {
        if (isAtTarget()) {
            setState(SpindexerState.IDLE);
        } else {
            // Determine shortest path to target
            double error = targetPosition - getCurrentPosition();
            double direction = Math.signum(error);
            spindexerMotor.setPower(direction * ROTATION_POWER);
        }
    }

    private void handleShootingState() {
        spindexerMotor.setPower(SHOOT_POWER);
    }

    private void handleCWState() {
        if (!cwInitialized) {
            targetSlot++;
            if (targetSlot > 3) targetSlot = 1;

            updateTargetPosition();
            cwInitialized = true;
        }

        setState(SpindexerState.POSITIONING);
    }

    private void handleCCWState() {
        spindexerMotor.setPower(-ROTATION_POWER);
    }

    private boolean cwInitialized = false;

    private void onStateChange(SpindexerState newState) {
        if (newState == SpindexerState.IDLE) {
            spindexerMotor.setPower(0.0);
        }
        if (newState != SpindexerState.CW) {
            cwInitialized = false;
        }
    }

    private void updateTargetPosition() {
        switch (targetSlot) {
            case 1:
                targetPosition = SLOT_1_POSITION;
                break;
            case 2:
                targetPosition = SLOT_2_POSITION;
                break;
            case 3:
                targetPosition = SLOT_3_POSITION;
                break;
        }
    }

    private boolean isGreen(ColorSensor sensor) {
        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();

        // Convert RGB to HSV for better color detection
        float[] hsv = new float[3];
        android.graphics.Color.RGBToHSV(red, green, blue, hsv);

        float hue = hsv[0];
        float saturation = hsv[1] * 255;
        float value = hsv[2] * 255;

        // Check if color is within green range
        return (hue >= GREEN_HUE_MIN && hue <= GREEN_HUE_MAX) &&
                (saturation >= MIN_SATURATION) &&
                (value >= MIN_VALUE);
    }
}