package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Globals;

@TeleOp(name = "Solo RED 🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴")
public class Tele_Op_Solo_Red extends Tele_Op_Solo {

    @Override
    public void initialize() {
        super.initialize();
        Globals.ALLIANCE = Globals.COLORS.RED;
    }
}