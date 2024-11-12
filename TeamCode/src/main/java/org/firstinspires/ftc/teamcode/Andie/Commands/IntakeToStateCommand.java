package org.firstinspires.ftc.teamcode.Andie.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Andie.Subsystems.Extendo;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Intake;

public class IntakeToStateCommand  extends ParallelCommandGroup {

    public String intakeCurrentState = "";


    public IntakeToStateCommand(Extendo extendo, Intake intake, String state) {
        switch (state.toLowerCase()) {
            case "Out":
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(intake::intakeUp),
                                new InstantCommand(extendo::extendoOut)
                        )
                );
                break;
            case "Middle":
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(intake::intakeUp),
                                new InstantCommand(extendo::extendoMiddle)
                        )
                );
                break;
            case "In":
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(intake::intakeUp),
                                new WaitCommand(200),
                                new InstantCommand(extendo::extendoIn)
                        )
                );
                break;
        }
    }
}