package org.firstinspires.ftc.teamcode.Echo.Commands.IntakeCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Echo.Subsystems.Intake;



public class IntakeInCommand extends SequentialCommandGroup {


    public IntakeInCommand(Intake intake) {
        addCommands(
                new InstantCommand(intake::in),
                new WaitCommand(5000),
                new InstantCommand(intake::stop)
        );
    }
}
