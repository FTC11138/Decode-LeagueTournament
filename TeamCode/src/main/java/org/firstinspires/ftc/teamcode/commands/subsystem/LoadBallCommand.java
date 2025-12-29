package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.TurretSubsystem;

public class LoadBallCommand extends InstantCommand {
    public LoadBallCommand() {
        super(
                () -> Robot.getInstance().spindexerTestSubsystem.rotate120CCW()
        );
    }
}