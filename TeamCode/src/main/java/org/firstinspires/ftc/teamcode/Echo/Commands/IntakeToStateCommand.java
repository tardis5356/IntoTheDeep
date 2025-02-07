package org.firstinspires.ftc.teamcode.Echo.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Echo.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Wrist;

public class IntakeToStateCommand extends ParallelCommandGroup {
    //This beefy program is a file that holds a bunch of sequences for the lift, wrist, gripper, and arm
    //each sequence is its own case. they are all contained in a switch statement, that takes in the string 'desiredState', and then runs the case with the matching name
    //format is: case "name": addCommands(new Sequential/ParallelCommandGroup(new _Command, ... , _Command)); break;
    //in here the naming scheme of the cases is the currentRobotState To desiredRobotState. That pretty much describes what each case does

    public String depositCurrentState = "";
    boolean wasRaised;
    boolean IntakeToggle;
    boolean outaking;
    public IntakeToStateCommand(Intake intake, String desiredState/*, String setState*/) {
        switch (desiredState) {
            case "intakeDown":
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> intake.sIT.setPosition(BotPositions.INTAKE_WRIST_DOWN)),
                                new WaitCommand(200),
                                new InstantCommand(() -> intake.sIG.setPosition(BotPositions.INTAKE_ARM_DOWN)),
                                new InstantCommand(() -> wasRaised = false)
                        )
                );
                //setState = "intake";
                break;
            case "intakeUp":
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> intake.sIG.setPosition(BotPositions.INTAKE_WRIST_DOWN)),
                                new WaitCommand(200),
                                new InstantCommand(() -> intake.sIG.setPosition(BotPositions.INTAKE_ARM_UP)),
                                new InstantCommand(() -> wasRaised = true)
                        )
                );
                break;
            case "intakeOut":
                addCommands(
            new SequentialCommandGroup(
                    //new InstantCommand(()-> intake.samplePresent = false),
                    new InstantCommand(()-> outaking = true),
                    new InstantCommand(intake::out),
                    new InstantCommand(() -> IntakeToggle = false),
                    new WaitCommand(2000),
                    new InstantCommand(()-> outaking = false)
                    //new IntakeOutCommand(intake)
            )
                );
                break;
        }
    }    //These are no longer used.

}