package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

public class FieldCentricDrive {

    private final Follower follower;
    private double headingOffset = 0.0;
    private boolean fieldCentricEnabled = true;

    public FieldCentricDrive(Follower follower) {
        this.follower = follower;
    }

    public void resetFieldCentricHeading() {
        Pose pose = follower.getPose();
        headingOffset = pose == null ? 0.0 : pose.getHeading();
    }

    public void setFieldCentricEnabled(boolean enabled) {
        fieldCentricEnabled = enabled;
    }

    public boolean isFieldCentricEnabled() {
        return fieldCentricEnabled;
    }

    public double getHeadingOffset() {
        return headingOffset;
    }

    public void drive(Gamepad gamepad) {
        double forward = -gamepad.left_stick_y;
        double strafe = -gamepad.left_stick_x;
        double turn = -gamepad.right_stick_x;

        follower.setTeleOpDrive(
                forward,
                strafe,
                turn,
                !fieldCentricEnabled,
                fieldCentricEnabled ? headingOffset : 0.0
        );
    }
}
