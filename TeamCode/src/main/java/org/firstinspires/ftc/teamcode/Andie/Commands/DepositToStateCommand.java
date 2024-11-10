package org.firstinspires.ftc.teamcode.Andie.Commands;

import static org.firstinspires.ftc.teamcode.Andie.Subsystems.BotPositions.LIFT_WALL;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Andie.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Wrist;

public class DepositToStateCommand extends ParallelCommandGroup {

    public String currentState = "";


    public DepositToStateCommand(Arm arm, Wrist wrist, Gripper gripper, Lift lift, String state) {
        switch (state.toLowerCase()) {
            case "basketToIntake":
                currentState = "intake";
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(wrist::wristIntake),
                                new InstantCommand(arm::armIntake),
                                new WaitCommand(400),
                                new InstantCommand(gripper::intakeGripper),
                                new LiftToStateCommand(lift, 0, 25)
                                //new InstantCommand(wrist::wristIntake)

                        )
                );
                break;
            case "specimenToIntake":
                currentState = "intake";
                addCommands(
                        new SequentialCommandGroup(
                                new LiftToStateCommand(lift, 10, 25),
                                new InstantCommand(wrist::wristTuck),
                                new WaitCommand(10),
                                new InstantCommand(arm::armIntake),
                                new WaitCommand(400),
                                new InstantCommand(wrist::wristIntake),
                                new InstantCommand(gripper::intakeGripper),
                                new LiftToStateCommand(lift, 0, 25)

                        )
                );
                break;
            case "intakeToSpecimen":
                currentState = "specimen";
                addCommands(
                        new SequentialCommandGroup(
                                new LiftToStateCommand(lift, 10, 25),
                                new InstantCommand(wrist::wristTuck),
                                new InstantCommand(arm::armSpecimen),
                                new WaitCommand(100),
                                new InstantCommand(wrist::wristSpecimen)
                        )
                );
                break;
            case "basketToSpecimen":
                currentState = "specimen";
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(wrist::wristTuck),
                                new WaitCommand(250),
                                new InstantCommand(arm::armSpecimen),
                                new WaitCommand(500),
                                new InstantCommand(wrist::wristSpecimen)
                        )
                );
                break;
            case "intakeToBasket":
                currentState = "basket";
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(wrist::wristTuck),
                                new WaitCommand(250),
                                new InstantCommand(wrist::wristBasket),
                                new InstantCommand(arm::armBasket)
                        )
                );
                break;
            case "transit":
                currentState = "transit";
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(wrist::wristTuck),
                                new InstantCommand(arm::armTransit),
                                new LiftToStateCommand(lift, LIFT_WALL, 25)
                        )
                );
                break;

        }
    }
}