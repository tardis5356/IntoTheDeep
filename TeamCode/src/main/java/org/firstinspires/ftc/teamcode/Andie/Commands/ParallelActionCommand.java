package org.firstinspires.ftc.teamcode.Andie.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Andie.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.TestBed.ActionCommand;


public class ParallelActionCommand extends ParallelCommandGroup {

    public String depositCurrentState = "";

    private ActionCommand RedSpec_StartToSub;
    private ActionCommand RedSpec_SubToLeftSpec;
    private ActionCommand RedSpec_RightSpecToObs;
    private ActionCommand RedSpec_SpecDepoToObs;
    private ActionCommand RedSpec_ObsToRightSpec;
    private ActionCommand RedSpec_LeftSpecToObs;
    private ActionCommand RedSpec_LeftSpecToMidWay;
    private ActionCommand RedSpec_MidSpecToObs;
    private ActionCommand RedSpec_ObsToMidSpec;
    private ActionCommand RedSpec_ObsToSub;
    private ActionCommand RedSpec_SubToObs;

    private ActionCommand RedSpec_ObsSpecCheck;
    static String DepositState;



    public ParallelActionCommand (Arm arm, Wrist wrist, Gripper gripper, Lift lift, String desiredState/*, String setState*/) {
        switch (desiredState) {
            case "redSpec_StartToSub":
                addCommands(
                        new ParallelCommandGroup(
                                new InstantCommand(gripper::close),
                                new InstantCommand(wrist::specimen),
                                new InstantCommand(arm::specimen),
                                RedSpec_StartToSub
                        )
                );

                break;

            case "scoreSpecimen":
                addCommands(
                        new SequentialCommandGroup(
                                new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH - 1030, 50),
                                new InstantCommand(gripper::open),
                                new LiftToStateCommand(lift, BotPositions.LIFT_TRANSIT, BotPositions.LIFT_TOLERANCE)
                                )
                );
                break;

            case "redSpec_SubToLeftSpec":
                addCommands(
                        new ParallelCommandGroup(
                                RedSpec_SubToLeftSpec,
                                new DepositToStateCommand(arm, wrist, gripper, lift, "specimenToWall"),
                                new InstantCommand(() -> DepositState = "wall")
                        )
                );
                break;

            case "RightSpecPickUpSpecimen":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                    RedSpec_RightSpecToObs,
                                    new LiftToStateCommand(lift, BotPositions.LIFT_WALL, BotPositions.LIFT_TOLERANCE)
                        ),
                                new InstantCommand(gripper::close)
                        )
                );
                break;

            case "specDepoToObs":
                addCommands(
                        new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                RedSpec_SpecDepoToObs,
                                new LiftToStateCommand(lift, BotPositions.LIFT_WALL, BotPositions.LIFT_TOLERANCE)
                        ),
                                RedSpec_ObsSpecCheck,
                        new WaitCommand(1500),
                        new InstantCommand(gripper::close)
                )
                );
                break;

            case "ObsToSub":
                addCommands(
                        new SequentialCommandGroup(
                                RedSpec_ObsToSub,
                                new DepositToStateCommand(arm, wrist, gripper, lift, "wallToSpecimen"),
                                new InstantCommand(() -> DepositState = "specimen")),
                        new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH - 1000, BotPositions.LIFT_TOLERANCE),
                        new InstantCommand(gripper::open)
                );
                break;

            case "redSpec_Park":
                addCommands(
                        new SequentialCommandGroup(
                                RedSpec_SubToObs

                        )
                );
                break;

        }
    }
    public void setBasket(){
        depositCurrentState = "basket";
    }
    public void setIntake(){
        depositCurrentState = "intake";
    }
    public void setWall(){
        depositCurrentState = "wall";
    }
}