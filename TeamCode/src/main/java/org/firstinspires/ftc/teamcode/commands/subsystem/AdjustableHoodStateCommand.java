package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ShooterSubsystem;

public class AdjustableHoodStateCommand extends InstantCommand {
    public AdjustableHoodStateCommand(ShooterSubsystem.AdjHoodState state) {
        super(
                () -> Robot.getInstance().shooterSubsystem.updateAdjHoodState(state)
        );
    }
}