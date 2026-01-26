package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.TurretOdometrySubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.TurretSubsystem;

public class TurretStateCommand extends InstantCommand {
    public TurretStateCommand(TurretOdometrySubsystem.TurretState state) {
        super(
                () -> Robot.getInstance().turretOdometrySubsystem.setTurretState(state)
        );
    }
}