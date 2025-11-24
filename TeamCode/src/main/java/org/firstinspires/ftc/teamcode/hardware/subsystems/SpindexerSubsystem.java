package org.firstinspires.ftc.teamcode.hardware.subsystems;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;

import java.util.Arrays;
import java.util.List;

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
    private final DcMotorEx spindexerMotor;
    private final ColorSensor[][] slotSensors = new ColorSensor[3][2];

    // === Encoder / motion tuning (TODO: tune on robot) ===
    // TICKS_PER_SLOT = absolute encoder tick change for ONE 120° step CW.
    private static final double TICKS_PER_SLOT = 1000.0; // TODO: tune this
    private static final double POSITION_TOLERANCE_TICKS = 50.0;

    // Motor powers (can be tuned)
    private static final double CW_POWER  = 0.4;  // shooting direction
    private static final double CCW_POWER = -0.4; // sorting/loading direction

    // === Enums ===

    public enum Slot {
        SLOT0(0),
        SLOT1(1),
        SLOT2(2);

        public final int index;
        Slot(int i) { this.index = i; }

        public static Slot fromIndex(int idx) {
            int i = ((idx % 3) + 3) % 3;
            for (Slot s : values()) {
                if (s.index == i) return s;
            }
            return SLOT0;
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
        MOVING,
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

    private Slot frontSlot = Slot.SLOT0;             // which slot is at the launcher
    private final SlotInfo[] slotInfos = new SlotInfo[3];

    private SpindexerState state = SpindexerState.IDLE;
    private double targetTicks = Double.NaN;

    public SpindexerSubsystem(HardwareMap hardwareMap,
                              String motorName,
                              String slot0SensorAName, String slot0SensorBName,
                              String slot1SensorAName, String slot1SensorBName,
                              String slot2SensorAName, String slot2SensorBName) {

        this.spindexerMotor = hardwareMap.get(DcMotorEx.class, motorName);

        // Color sensors (2 per slot)
        slotSensors[0][0] = hardwareMap.get(ColorSensor.class, slot0SensorAName);
        slotSensors[0][1] = hardwareMap.get(ColorSensor.class, slot0SensorBName);
        slotSensors[1][0] = hardwareMap.get(ColorSensor.class, slot1SensorAName);
        slotSensors[1][1] = hardwareMap.get(ColorSensor.class, slot1SensorBName);
        slotSensors[2][0] = hardwareMap.get(ColorSensor.class, slot2SensorAName);
        slotSensors[2][1] = hardwareMap.get(ColorSensor.class, slot2SensorBName);

        // Init slot info
        slotInfos[0] = new SlotInfo(Slot.SLOT0, BallColor.UNKNOWN);
        slotInfos[1] = new SlotInfo(Slot.SLOT1, BallColor.UNKNOWN);
        slotInfos[2] = new SlotInfo(Slot.SLOT2, BallColor.UNKNOWN);

        // Motor setup
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Assume starting aligned with SLOT0 at front
        targetTicks = getCurrentTicks();
        state = SpindexerState.IDLE;

        Robot.getInstance().subsystems.add(this);
    }

    // === Public getters ===

    public SpindexerState getState() {
        return state;
    }

    public Slot getFrontSlot() {
        return frontSlot;
    }

    public List<SlotInfo> getSlotInfos() {
        return Arrays.asList(slotInfos);
    }

    public BallColor getBallColor(Slot slot) {
        return slotInfos[slot.index].color;
    }

    public double getCurrentTicks() {
        return spindexerMotor.getCurrentPosition();
    }

    // === Homing helper ===

    /**
     * If you manually align a certain slot in front of the launcher,
     * call this to reset encoder & logical front slot.
     */
    public void homeCurrentPositionToSlot(Slot slot) {
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontSlot = slot;
        targetTicks = getCurrentTicks();
        state = SpindexerState.IDLE;
    }

    // === High-level movement API ===

    /** Advance one slot in CW direction (shooting). */
    public void advanceOneSlotCWForShooting() {
        if (state != SpindexerState.IDLE) return;

        state = SpindexerState.SHOOTING_CW;

        // Update logical front slot
        int nextIndex = (frontSlot.index + 1) % 3;
        frontSlot = Slot.fromIndex(nextIndex);

        // Move motor one "slot" worth of ticks CW
        double current = getCurrentTicks();
        targetTicks = current + TICKS_PER_SLOT;
        spindexerMotor.setPower(CW_POWER);
    }

    /** Advance one slot in CCW direction (sorting/loading). */
    public void advanceOneSlotCCWForSorting() {
        if (state != SpindexerState.IDLE) return;

        state = SpindexerState.SORTING_CCW;

        // Update logical front slot
        int nextIndex = (frontSlot.index - 1 + 3) % 3;
        frontSlot = Slot.fromIndex(nextIndex);

        // Move motor one "slot" worth of ticks CCW
        double current = getCurrentTicks();
        targetTicks = current - TICKS_PER_SLOT;
        spindexerMotor.setPower(CCW_POWER);
    }

    /** Stop motion & hold current slot. */
    public void stop() {
        spindexerMotor.setPower(0.0);
        state = SpindexerState.IDLE;
        targetTicks = getCurrentTicks();
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
        if (Double.isNaN(targetTicks)) return false;
        return abs(getCurrentTicks() - targetTicks) <= POSITION_TOLERANCE_TICKS;
    }

    // === RE_SubsystemBase hooks ===

    @Override
    public void updateData() {
        // Hook telemetry struct if you want
        // Robot.getInstance().data.spindexerFrontSlot = frontSlot;
        // Robot.getInstance().data.spindexerState = state;
        // Robot.getInstance().data.spindexerSlot0Color = slotInfos[0].color;
    }

    @Override
    public void periodic() {
        // Handle motion completion
        switch (state) {
            case SHOOTING_CW:
            case SORTING_CCW:
            case MOVING:
                if (isAtTarget()) {
                    spindexerMotor.setPower(0.0);
                    state = SpindexerState.IDLE;
                }
                break;
            case IDLE:
            default:
                break;
        }

        // Continuously update slot colors
        updateSlotColors();
    }
}