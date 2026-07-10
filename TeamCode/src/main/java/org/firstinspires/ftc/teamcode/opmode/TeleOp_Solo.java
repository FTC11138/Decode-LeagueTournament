package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.util.Globals;

@TeleOp(name = "TeleOp Solo", group = "TeleOp")
public class TeleOp_Solo extends OpMode {

    private Robot robot;
    private FieldCentricDrive drive;
    private boolean previousTouchpad;

    @Override
    public void init() {
        Globals.IS_AUTO = false;

        robot = Robot.getInstance();
        robot.initialize(hardwareMap, telemetry);

        drive = new FieldCentricDrive(robot.follower);
        drive.setFieldCentricEnabled(true);
        drive.resetFieldCentricHeading();
    }

    @Override
    public void loop() {
        if (gamepad1.touchpad && !previousTouchpad) {
            drive.resetFieldCentricHeading();
        }
        previousTouchpad = gamepad1.touchpad;

        drive.drive(gamepad1);

        robot.periodic();
        robot.updateData();

        telemetry.addData("Field Centric", drive.isFieldCentricEnabled());
        telemetry.addData("Field Heading Offset", Math.toDegrees(drive.getHeadingOffset()));
        robot.write();
        telemetry.update();
    }

    @Override
    public void stop() {
        if (robot != null && robot.follower != null) {
            robot.follower.setTeleOpDrive(0.0, 0.0, 0.0, false, 0.0);
            robot.periodic();
        }
    }
}
