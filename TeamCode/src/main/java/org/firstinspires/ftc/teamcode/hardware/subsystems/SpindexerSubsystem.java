package org.firstinspires.ftc.teamcode.hardware.subsystems;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;

import java.util.Arrays;
import java.util.List;



/*
 *  ===================== SPINDEXER SLOT MAP =====================
 *
 *  Slots are FIXED positions on the robot.
 *  The plate rotates, but the slots do not move.
 *
 *                 FRONT OF ROBOT
 *                    (Shooter)
 *
 *                        ^
 *                        |
 *                        |
 *                    [ SLOT_1 ]
 *                     (Front)
 *
 *               /                   \
 *              /                     \
 *             /                       \
 *
 *      [ SLOT_3 ]                 [ SLOT_2 ]
 *     (Back Left)                (Back Right)
 *
 *
 *  Rotation Conventions:
 *  ----------------------
 *  - CW  rotation (negative direction) shoots balls:
 *        SLOT_1 → SLOT_3 → SLOT_2 → SLOT_1 → ...
 *
 *  - CCW rotation (positive direction) moves balls:
 *        SLOT_1 → SLOT_2 → SLOT_3 → SLOT_1 → ...
 *
 *  Encoder Model:
 *  --------------
 *  - Each full rotation = TICKS_PER_REV encoder ticks.
 *  - Each slot is exactly 120° apart → TICKS_PER_SLOT = TICKS_PER_REV / 3.
 *
 *  Examples:
 *      rotateBallToShooter(SLOT_3)
 *          SLOT_3 → SLOT_2 → SLOT_1 (2 CCW steps)
 *
 *      rotateBallToShooter(SLOT_2)
 *          SLOT_2 → SLOT_1 (1 CCW step)
 *
 *  ================================================================
 */


/**
 * Spindexer driven by a motor with encoder.
 *
 * - 3 slots in a triangle (120° apart).
 * - Motor with encoder rotates the plate.
 * - 2 color sensors per slot (6 total).
 *
 * CW = shooting direction.
 * CCW = sorting/loading direction.
 */
public class SpindexerSubsystem extends RE_SubsystemBase {

    // === Hardware ===
    private final DcMotorEx spindexerEncoder;
    private final CRServo spindexerServo;
    private final ColorSensor[][] slotSensors;

    // === Encoder / motion tuning (TODO: tune on robot) ===
    // TICKS_PER_SLOT = absolute encoder tick change for ONE 120° step CW.
    private static final int TICKS_PER_REV = 1000; // TODO: tune this
    private static final int TICKS_PER_SLOT = TICKS_PER_REV / 3;
    private static final int POSITION_TOLERANCE_TICKS = 50;

    // Motor powers (can be tuned)
    private static final double CW_POWER  = -1;  // shooting direction
    private static final double CCW_POWER = 1; // sorting/loading direction
    private static final double LOAD_POWER = 1;
    private static final double STOP_POWER = 0;

    // === Enums ===

    public enum Slot {
        SLOT_1(0),
        SLOT_2(1),
        SLOT_3(2);

        public final int index;
        Slot(int i) { this.index = i; }

        public static Slot fromIndex(int idx) {
            int i = ((idx % 3) + 3) % 3;
            for (Slot s : values()) {
                if (s.index == i) return s;
            }
            return SLOT_1;
        }
    }

    public enum BallColor {
        GREEN,
        PURPLE,
        NONE,
        UNKNOWN
    }

    public enum SpindexerState {
        IDLE,
        LOADING,
        SHOOTING_CW,
        SORTING_CCW
    }

    public static class SlotInfo {
        public final Slot slot;
        public BallColor color;

        public SlotInfo(Slot slot, BallColor color) {
            this.slot = slot;
            this.color = color;
        }
    }

    // === State ===

    private final SlotInfo[] slotInfos;

    private SpindexerState state;
    private int targetPos;

    public SpindexerSubsystem(HardwareMap hardwareMap,
                              String encoderName, String servoName,
                              String slot0SensorAName, String slot0SensorBName,
                              String slot1SensorAName, String slot1SensorBName,
                              String slot2SensorAName, String slot2SensorBName) {

        this.spindexerEncoder = hardwareMap.get(DcMotorEx.class, encoderName);
        this.spindexerServo = hardwareMap.get(CRServo.class, servoName);


        // Color sensors (2 per slot)
        slotSensors = new ColorSensor[3][2];
        slotSensors[0][0] = hardwareMap.get(ColorSensor.class, slot0SensorAName);
        slotSensors[0][1] = hardwareMap.get(ColorSensor.class, slot0SensorBName);
        slotSensors[1][0] = hardwareMap.get(ColorSensor.class, slot1SensorAName);
        slotSensors[1][1] = hardwareMap.get(ColorSensor.class, slot1SensorBName);
        slotSensors[2][0] = hardwareMap.get(ColorSensor.class, slot2SensorAName);
        slotSensors[2][1] = hardwareMap.get(ColorSensor.class, slot2SensorBName);

        // Init slot info
        slotInfos = new SlotInfo[3];
        slotInfos[0] = new SlotInfo(Slot.SLOT_1, BallColor.UNKNOWN);
        slotInfos[1] = new SlotInfo(Slot.SLOT_2, BallColor.UNKNOWN);
        slotInfos[2] = new SlotInfo(Slot.SLOT_3, BallColor.UNKNOWN);

        // Motor setup
        spindexerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        state = SpindexerState.IDLE;
        targetPos = 0;

        Robot.getInstance().subsystems.add(this);
    }

    public SpindexerState getSpindexerState() {
        return this.state;
    }

    public int getTargetPos() {
        return this.targetPos;
    }

    public List<SlotInfo> getSlotInfos() {
        return Arrays.asList(slotInfos);
    }

    public BallColor getBallColor(Slot slot) {
        return slotInfos[slot.index].color;
    }

    public int getCurrentPos() {
        return spindexerEncoder.getCurrentPosition();
    }



    // === High-level movement API ===

    /** Advance one slot in CW direction (shooting). */
    public void shootSingleBall() {
        if (state != SpindexerState.IDLE) return;

        state = SpindexerState.SHOOTING_CW;

        // Move motor one "slot" worth of ticks CW
        int current = getCurrentPos();
        targetPos = current + (TICKS_PER_SLOT * (int) Math.ceil(CW_POWER));
    }

    /** Shoots all balls by rotating 1 revolution CW */
    public void shootAllBalls() {
        if (state != SpindexerState.IDLE) return;

        state = SpindexerState.SHOOTING_CW;

        // Move motor one "slot" worth of ticks CW
        int current = getCurrentPos();
        targetPos = current + (TICKS_PER_REV * (int) Math.ceil(CW_POWER));
    }

    /** Advance one slot in CCW direction (loading). */
    public void loadSingleBall() {
        if (state != SpindexerState.IDLE) return;

        state = SpindexerState.LOADING;

        // Move motor one "slot" worth of ticks CCW
        int current = getCurrentPos();
        targetPos = current + (TICKS_PER_SLOT * (int) Math.ceil(LOAD_POWER));
    }

    /**
     * Rotate CCW (sorting direction) so that the given slot ends up
     * at the shooting position in one continuous move (0–2 slots CCW).
     */
    public void rotateSlotToShoot(Slot desiredFront) {
        assert desiredFront != null;

        if (state != SpindexerState.IDLE) return;

        state = SpindexerState.SORTING_CCW;

        int current = getCurrentPos();
        targetPos = current + (desiredFront.index * TICKS_PER_SLOT * (int) Math.ceil(CCW_POWER));
    }


    public Slot getGreenBallSlot() {
        for (Slot slot : Slot.values()) {
            if (getBallColor(slot) == BallColor.GREEN) return slot;
        }
        return null;
    }



    /** Stop motion & hold current slot. */
    public void stop() {
        spindexerServo.setPower(0.0);
        state = SpindexerState.IDLE;
        targetPos = getCurrentPos();
    }

    // === Color logic ===

    private void updateSlotColors() {
        for (Slot slot : Slot.values()) {
            ColorSensor sensorA = slotSensors[slot.index][0];
            ColorSensor sensorB = slotSensors[slot.index][1];

            BallColor a = detectColor(sensorA);
            BallColor b = detectColor(sensorB);

            slotInfos[slot.index].color = combineColors(a, b);
        }
    }

    private BallColor detectColor(ColorSensor sensor) {
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        int sum = r + g + b;
        if (sum < 50) {
            return BallColor.NONE;
        }

        // Placeholder logic — tune these with telemetry!
        if (g > r && g > b) {
            return BallColor.GREEN;
        } else if (r > g && b > g) {
            return BallColor.PURPLE;
        } else {
            return BallColor.UNKNOWN;
        }
    }

    private BallColor combineColors(BallColor a, BallColor b) {
        if (a == b && a != BallColor.UNKNOWN) return a;
        if (a == BallColor.NONE && (b == BallColor.GREEN || b == BallColor.PURPLE)) return b;
        if (b == BallColor.NONE && (a == BallColor.GREEN || a == BallColor.PURPLE)) return a;
        if (a == BallColor.NONE && b == BallColor.NONE) return BallColor.NONE;
        return BallColor.UNKNOWN;
    }

    // === Internal helpers ===

    private boolean isAtTarget() {
        return abs(getCurrentPos() - targetPos) <= POSITION_TOLERANCE_TICKS;
    }

    private void updateServoPower() {
        switch (state) {
            case SHOOTING_CW:
                spindexerServo.setPower(CW_POWER);
                break;
            case LOADING:
                spindexerServo.setPower(CCW_POWER);
                break;
            case IDLE:
            default:
                spindexerServo.setPower(STOP_POWER);
                break;
        }
    }

    // === RE_SubsystemBase hooks ===

    @Override
    public void updateData() {
        // Robot.getInstance().data.spindexerState      = state;
        // Robot.getInstance().data.spindexerTargetPos  = targetPos;
        // Robot.getInstance().data.spindexerCurrentPos = getCurrentPos();
    }

    @Override
    public void periodic() {
        // Drive servo based on current state
        updateServoPower();

        // Check if we've reached the target encoder position
        if (state != SpindexerState.IDLE) {
            if (isAtTarget()) {
                stop(); // sets state = IDLE and stops servo
            }
        }

        // Continuously update colors
        updateSlotColors();
    }


}