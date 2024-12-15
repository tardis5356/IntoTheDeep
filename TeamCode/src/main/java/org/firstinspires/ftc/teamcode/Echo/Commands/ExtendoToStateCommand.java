package org.firstinspires.ftc.teamcode.Echo.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Echo.Subsystems.Extendo;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Intake;

public class ExtendoToStateCommand extends ParallelCommandGroup {
    private IntakeInCommand intakeInCommand;

    public ExtendoToStateCommand(Intake intake, Extendo extendo, String state) {
        switch (state) {
            case "extendoOut":
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(extendo::out),
                                new InstantCommand(intake::down),
                                new InstantCommand(intake::in),
                                intakeInCommand
                        )
                );

            case "extendoIn":
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(intake::up),
                                new WaitCommand(300),
                                new InstantCommand(extendo::in),
                                intakeInCommand

                        )
                );
                break;

        }
    }
}