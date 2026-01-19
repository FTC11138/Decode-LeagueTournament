package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Globals;

@TeleOp(name = "Solo BLUE 🔵🔵🔵🔵🔵🔵🔵🔵🔵🔵🔵🔵")
public class Tele_Op_Solo_Blue extends Tele_Op_Solo_Test {

    @Override
    public void initialize() {
        super.initialize();
        Globals.ALLIANCE = Globals.COLORS.BLUE;
    }
}