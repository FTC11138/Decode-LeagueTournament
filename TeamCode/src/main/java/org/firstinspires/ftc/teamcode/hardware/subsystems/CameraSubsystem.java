package org.firstinspires.ftc.teamcode.hardware.subsystems;

import static org.firstinspires.ftc.teamcode.util.Globals.ALLIANCE;
import static java.lang.Math.abs;
import static java.lang.Math.toRadians;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;

import java.util.List;

public class CameraSubsystem extends RE_SubsystemBase {

    private final Limelight3A limelight;

    private LLResult limelightResult;
    private Obelisk obeliskResult;
    private ShootDistance shootDistance;

    // Camera geometry parameters
    private final double minRange = 1.0;
    private final double maxRange = 3.0;
    private final double CAMERA_HEIGHT_M = 0.39267;
    private final double TAG_HEIGHT_M = 1.22;
    private final double CAMERA_PITCH_RAD = toRadians(20.0);

    // Current turret/camera yaw angle
    private double currentCameraYawDeg = 0.0;

    private LLResultTypes.FiducialResult bestBasketTarget = null;
    private double distanceM = Double.NaN;

    public enum ShootDistance {
        INRANGE,
        OUTOFRANGE
    }

    public enum Obelisk {
        PPG,
        PGP,
        GPP,
        PPP
    }

    public CameraSubsystem(HardwareMap hardwareMap, String limelightName) {
        this.limelight = hardwareMap.get(Limelight3A.class, limelightName);
        obeliskResult = Obelisk.PPP;
        this.shootDistance = ShootDistance.OUTOFRANGE;
        this.currentCameraYawDeg = 0.0;

        limelight.setPollRateHz(100);
        limelight.start();

        Robot.getInstance().subsystems.add(this);
    }

    // ==================== PUBLIC API ====================

    public boolean hasBasket() {
        return bestBasketTarget != null;
    }

    public double getBasketYawDeg() {
        if (bestBasketTarget != null) {
            return bestBasketTarget.getTargetXDegrees();
        }
        if (limelightResult != null && limelightResult.isValid()) {
            return limelightResult.getTx();
        }
        return Double.NaN;
    }

    public double getBasketDistanceM() {
        return distanceM;
    }

    public ShootDistance getShootDistance() {
        return shootDistance;
    }

    public Obelisk getObelisk() {
        return obeliskResult;
    }

    /**
     * Set current turret/camera yaw angle.
     * CRITICAL: TurretSubsystem calls this every cycle!
     */
    public void setCurrentCameraYaw(double yawDeg) {
        this.currentCameraYawDeg = yawDeg;
    }

    public double getCurrentCameraYaw() {
        return currentCameraYawDeg;
    }

    @Override
    public void updateData() {
        Robot.getInstance().data.obelisk = obeliskResult;
    }

    @Override
    public void periodic() {
        limelightResult = limelight.getLatestResult();

        bestBasketTarget = null;
        distanceM = Double.NaN;

        if (limelightResult == null || !limelightResult.isValid()) return;

        List<LLResultTypes.FiducialResult> fiducials = limelightResult.getFiducialResults();
        if (fiducials != null) {
            for (LLResultTypes.FiducialResult target : fiducials) {

                // Obelisk detection (tags 21, 22, 23)
                switch (target.getFiducialId()) {
                    case 21:
                        obeliskResult = Obelisk.GPP;
                        break;
                    case 22:
                        obeliskResult = Obelisk.PGP;
                        break;
                    case 23:
                        obeliskResult = Obelisk.PPG;
                        break;
                    default:
                        break;
                }

                // Basket detection (tag 20 for Blue, tag 24 for Red)
                boolean isAllianceBasket =
                        (ALLIANCE == Globals.COLORS.BLUE && target.getFiducialId() == 20) ||
                                (ALLIANCE == Globals.COLORS.RED  && target.getFiducialId() == 24);

                if (isAllianceBasket) {
                    if (bestBasketTarget == null) {
                        bestBasketTarget = target;
                    } else {
                        // Pick the basket closest to center (smallest tx)
                        if (abs(target.getTargetXDegrees()) < abs(bestBasketTarget.getTargetXDegrees())) {
                            bestBasketTarget = target;
                        }
                    }
                }
            }
        }

        // Calculate distance to basket
        Double tyDeg = null;
        if (bestBasketTarget != null) {
            tyDeg = bestBasketTarget.getTargetYDegrees();
        } else if (limelightResult.isValid()) {
            tyDeg = limelightResult.getTy();
        }

        if (tyDeg != null) {
            double tyRad = toRadians(tyDeg);
            double denom = Math.tan(CAMERA_PITCH_RAD + tyRad);
            if (abs(denom) > 1e-6) {
                distanceM = (TAG_HEIGHT_M - CAMERA_HEIGHT_M) / denom;
            } else {
                distanceM = Double.NaN;
            }
        }

        // Determine if we're in shooting range
        if (!Double.isNaN(distanceM) && distanceM >= minRange && distanceM <= maxRange) {
            shootDistance = ShootDistance.INRANGE;
        } else {
            shootDistance = ShootDistance.OUTOFRANGE;
        }

        Robot.getInstance().data.obelisk = obeliskResult;
    }
}