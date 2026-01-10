package org.firstinspires.ftc.teamcode.hardware.subsystems;

import static org.firstinspires.ftc.teamcode.util.Globals.ALLIANCE;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;

import java.util.ArrayList;
import java.util.List;

public class CameraSubsystem extends RE_SubsystemBase {

    private final Limelight3A limelight;
    private final Follower follower;

    private LLResult limelightResult;
    private Obelisk obeliskResult;
    private ShootDistance shootDistance;
    private CameraState cameraState;

    // Camera geometry parameters
    private final double minRange = 1.0;
    private final double maxRange = 3.0;
    private final double CAMERA_HEIGHT_M = 0.39267;
    private final double TAG_HEIGHT_M = 1.22;
    private final double CAMERA_PITCH_RAD = toRadians(0.0);

    // Turret position from robot center (in inches)
    private final double TURRET_OFFSET_X = -55.603/25.4;  // ~-2.19" behind center
    private final double TURRET_OFFSET_Y = 0.0;           // On centerline

    // Camera position from turret pivot (in inches) - MEASURE THESE!
    private final double CAMERA_FROM_TURRET_X = 0.0;  // Forward from turret pivot
    private final double CAMERA_FROM_TURRET_Y = 0.0;  // Left from turret pivot

    // Current turret/camera yaw angle
    private double currentCameraYawDeg = 0.0;

    // Known AprilTag positions on field (in inches)
    private static final class AprilTagPosition {
        double x, y, heading;
        AprilTagPosition(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    // INTO THE DEEP AprilTag positions
    private final AprilTagPosition[] TAG_POSITIONS = {
            new AprilTagPosition(16.2, 131.35, Math.toRadians(54)),
            new AprilTagPosition(127.8, 131.35, Math.toRadians(126)),
            new AprilTagPosition(72, 144, Math.toRadians(180)),
            new AprilTagPosition(72, 144, Math.toRadians(180)),
            new AprilTagPosition(72, 144, Math.toRadians(180)),
    };

    private LLResultTypes.FiducialResult bestBasketTarget = null;
    private double distanceM = Double.NaN;

    public enum CameraState {
        ON,
        OFF
    }

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

    public CameraSubsystem(HardwareMap hardwareMap, String limelightName, Follower follower) {
        this.limelight = hardwareMap.get(Limelight3A.class, limelightName);
        this.follower = follower;
        obeliskResult = Obelisk.PPP;
        this.shootDistance = ShootDistance.OUTOFRANGE;
        this.currentCameraYawDeg = 0.0;

        startCamera();
        Robot.getInstance().subsystems.add(this);
    }

    private void startCamera() {
        cameraState = CameraState.ON;
        limelight.setPollRateHz(100);
        limelight.start();
    }

    @SuppressWarnings("unused")
    private void stopCamera() {
        cameraState = CameraState.OFF;
        limelight.stop();
    }

    // ==================== LOCALIZATION FUNCTIONS ====================

    public boolean relocalizeFromAprilTags() {
        if (limelightResult == null || !limelightResult.isValid()) {
            return false;
        }

        List<LLResultTypes.FiducialResult> fiducials = limelightResult.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return false;
        }

        List<Pose> poseEstimates = new ArrayList<>();
        List<Double> poseWeights = new ArrayList<>();

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int tagId = fiducial.getFiducialId();
            if (getTagIndex(tagId) < 0) continue;

            Pose robotPose = calculateRobotPoseFromFiducial(fiducial);

            if (robotPose != null) {
                double weight = calculateFiducialWeight(fiducial);
                poseEstimates.add(robotPose);
                poseWeights.add(weight);
            }
        }

        if (poseEstimates.isEmpty()) {
            return false;
        }

        Pose finalPose = weightedAveragePose(poseEstimates, poseWeights);

        if (finalPose != null) {
            follower.setPose(finalPose);
            return true;
        }

        return false;
    }

    private double calculateFiducialWeight(LLResultTypes.FiducialResult fiducial) {
        double txDeg = Math.abs(fiducial.getTargetXDegrees());
        double tyDeg = Math.abs(fiducial.getTargetYDegrees());

        double angularScore = 1.0 / (1.0 + (txDeg * txDeg + tyDeg * tyDeg) / 100.0);

        double areaScore = 1.0;
        if (fiducial.getTargetArea() > 0) {
            areaScore = Math.min(fiducial.getTargetArea() / 2.0, 1.0);
        }

        return angularScore * areaScore;
    }

    private Pose weightedAveragePose(List<Pose> poses, List<Double> weights) {
        if (poses.isEmpty()) return null;

        double totalWeight = 0.0;
        double weightedX = 0.0;
        double weightedY = 0.0;
        double weightedCosHeading = 0.0;
        double weightedSinHeading = 0.0;

        for (int i = 0; i < poses.size(); i++) {
            Pose pose = poses.get(i);
            double weight = weights.get(i);

            weightedX += pose.getX() * weight;
            weightedY += pose.getY() * weight;
            weightedCosHeading += Math.cos(pose.getHeading()) * weight;
            weightedSinHeading += Math.sin(pose.getHeading()) * weight;
            totalWeight += weight;
        }

        if (totalWeight < 1e-6) return null;

        double avgX = weightedX / totalWeight;
        double avgY = weightedY / totalWeight;
        double avgHeading = Math.atan2(weightedSinHeading, weightedCosHeading);

        return new Pose(avgX, avgY, avgHeading);
    }

    private Pose calculateRobotPoseFromFiducial(LLResultTypes.FiducialResult fiducial) {
        int tagIndex = getTagIndex(fiducial.getFiducialId());
        if (tagIndex < 0) return null;

        AprilTagPosition tagPos = TAG_POSITIONS[tagIndex];

        double txDeg = fiducial.getTargetXDegrees();
        double tyDeg = fiducial.getTargetYDegrees();

        // Distance calculation using FIXED pitch
        double tyRad = Math.toRadians(tyDeg);
        double denom = Math.tan(CAMERA_PITCH_RAD + tyRad);

        if (Math.abs(denom) < 1e-6) {
            return null;
        }

        double distanceM = (TAG_HEIGHT_M - CAMERA_HEIGHT_M) / denom;

        if (distanceM <= 0 || distanceM > 5.0) {
            return null;
        }

        double distanceInches = distanceM * 39.3701;

        // Apply turret yaw rotation
        double effectiveTxDeg = txDeg + currentCameraYawDeg;
        double txRad = Math.toRadians(effectiveTxDeg);
        double horizontalOffset = distanceInches * Math.tan(txRad);

        double angleTagToCamera = tagPos.heading + Math.PI;

        double cameraX_field = tagPos.x
                + distanceInches * Math.cos(angleTagToCamera)
                - horizontalOffset * Math.sin(angleTagToCamera);

        double cameraY_field = tagPos.y
                + distanceInches * Math.sin(angleTagToCamera)
                + horizontalOffset * Math.cos(angleTagToCamera);

        double cameraHeading = angleTagToCamera - txRad;
        cameraHeading = normalizeAngle(cameraHeading);

        // Calculate robot heading (subtract turret rotation)
        double robotHeading = cameraHeading - Math.toRadians(currentCameraYawDeg);
        robotHeading = normalizeAngle(robotHeading);

        // Blend with odometry heading
        double currentHeading = follower.getPose().getHeading();
        double headingDiff = normalizeAngle(robotHeading - currentHeading);

        if (Math.abs(headingDiff) < Math.toRadians(30)) {
            robotHeading = normalizeAngle(currentHeading + headingDiff * 0.7);
        }

        // Calculate dynamic camera offset based on turret angle
        double turretAngleRad = Math.toRadians(currentCameraYawDeg);
        double cos_turret = Math.cos(turretAngleRad);
        double sin_turret = Math.sin(turretAngleRad);

        // Camera offset from robot center = turret offset + rotated camera-from-turret offset
        double cameraOffsetX = TURRET_OFFSET_X + (CAMERA_FROM_TURRET_X * cos_turret - CAMERA_FROM_TURRET_Y * sin_turret);
        double cameraOffsetY = TURRET_OFFSET_Y + (CAMERA_FROM_TURRET_X * sin_turret + CAMERA_FROM_TURRET_Y * cos_turret);

        // Transform from camera position to robot center
        double cos_robot = Math.cos(robotHeading);
        double sin_robot = Math.sin(robotHeading);

        double robotX = cameraX_field - (cameraOffsetX * cos_robot - cameraOffsetY * sin_robot);
        double robotY = cameraY_field - (cameraOffsetX * sin_robot + cameraOffsetY * sin_robot);

        return new Pose(robotX, robotY, robotHeading);
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private int getTagIndex(int tagId) {
        switch(tagId) {
            case 20: return 0;
            case 24: return 1;
            case 21: return 2;
            case 22: return 3;
            case 23: return 4;
            default: return -1;
        }
    }

    public LLResultTypes.FiducialResult getBestBasketTag() {
        return bestBasketTarget;
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
     * CRITICAL: Call this every time your turret moves!
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
        if (cameraState == CameraState.ON) {
            limelightResult = limelight.getLatestResult();
        }

        bestBasketTarget = null;
        distanceM = Double.NaN;

        if (limelightResult == null || !limelightResult.isValid()) return;

        List<LLResultTypes.FiducialResult> fiducials = limelightResult.getFiducialResults();
        if (fiducials != null) {
            for (LLResultTypes.FiducialResult target : fiducials) {

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

                boolean isAllianceBasket =
                        (ALLIANCE == Globals.COLORS.BLUE && target.getFiducialId() == 20) ||
                                (ALLIANCE == Globals.COLORS.RED  && target.getFiducialId() == 24);

                if (isAllianceBasket) {
                    if (bestBasketTarget == null) {
                        bestBasketTarget = target;
                    } else {
                        if (abs(target.getTargetXDegrees()) < abs(bestBasketTarget.getTargetXDegrees())) {
                            bestBasketTarget = target;
                        }
                    }
                }
            }
        }

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

        if (!Double.isNaN(distanceM) && distanceM >= minRange && distanceM <= maxRange) {
            shootDistance = ShootDistance.INRANGE;
        } else {
            shootDistance = ShootDistance.OUTOFRANGE;
        }

        Robot.getInstance().data.obelisk = obeliskResult;
    }
}