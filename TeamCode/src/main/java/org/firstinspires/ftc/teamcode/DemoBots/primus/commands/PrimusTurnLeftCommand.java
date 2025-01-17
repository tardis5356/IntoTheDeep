package org.firstinspires.ftc.teamcode.DemoBots.primus.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.DemoBots.optimus.Optimus_subsystems.OptimusDrive;
import org.firstinspires.ftc.teamcode.DemoBots.primus.Primus_subsystems.PrimusDrive;

public class PrimusTurnLeftCommand extends SequentialCommandGroup {
    public PrimusTurnLeftCommand(PrimusDrive drive) {
        addCommands(
                new InstantCommand(drive::TurnLeft)

        );
    }
}
