package org.firstinspires.ftc.teamcode.Echo.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Echo.Subsystems.Intake;



public class IntakeInCommand extends SequentialCommandGroup {


    public IntakeInCommand(Intake intake) {
        addCommands(
                new InstantCommand(intake::in)
        );
    }
}
