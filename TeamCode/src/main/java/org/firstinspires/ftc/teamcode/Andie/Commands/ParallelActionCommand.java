package org.firstinspires.ftc.teamcode.Andie.Commands;

import static org.firstinspires.ftc.teamcode.TestBed.AutoPathing.AutoTrajectories.redSpec_LeftSpecToMidWay;
import static org.firstinspires.ftc.teamcode.TestBed.AutoPathing.AutoTrajectories.redSpec_LeftSpecToObs;
import static org.firstinspires.ftc.teamcode.TestBed.AutoPathing.AutoTrajectories.redSpec_MidSpecToObs;
import static org.firstinspires.ftc.teamcode.TestBed.AutoPathing.AutoTrajectories.redSpec_ObsSpecCheck;
import static org.firstinspires.ftc.teamcode.TestBed.AutoPathing.AutoTrajectories.redSpec_ObsToMidSpec;
import static org.firstinspires.ftc.teamcode.TestBed.AutoPathing.AutoTrajectories.redSpec_ObsToRightSpec;
import static org.firstinspires.ftc.teamcode.TestBed.AutoPathing.AutoTrajectories.redSpec_ObsToSub;
//import static org.firstinspires.ftc.teamcode.TestBed.AutoPathing.AutoTrajectories.redSpec_RightSpecToObs;
import static org.firstinspires.ftc.teamcode.TestBed.AutoPathing.AutoTrajectories.redSpec_RightSpecToSub;
import static org.firstinspires.ftc.teamcode.TestBed.AutoPathing.AutoTrajectories.redSpec_SpecDepoToObs;
import static org.firstinspires.ftc.teamcode.TestBed.AutoPathing.AutoTrajectories.redSpec_StartToSub;
import static org.firstinspires.ftc.teamcode.TestBed.AutoPathing.AutoTrajectories.redSpec_SubToLeftSpec;
import static org.firstinspires.ftc.teamcode.TestBed.AutoPathing.AutoTrajectories.redSpec_SubToObs;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Andie.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Extendo;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.TestBed.ActionCommand;
import org.firstinspires.ftc.teamcode.TestBed.ExampleSubsystem;

import java.util.Set;


public class ParallelActionCommand extends ParallelCommandGroup {

    public String depositCurrentState = "";

    private ActionCommand RedSpec_StartToSub;
    private ActionCommand RedSpec_SubToLeftSpec;
    private ActionCommand RedSpec_RightSpecToSub;
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


    public ParallelActionCommand(Arm arm, Wrist wrist, Gripper gripper, Lift lift, ExampleSubsystem exampleSubsystem, String desiredState/*, String setState*/) {
        Set<Subsystem> requirements = Set.of(exampleSubsystem);

        RedSpec_StartToSub = new ActionCommand(redSpec_StartToSub, requirements);

        RedSpec_SubToLeftSpec = new ActionCommand(redSpec_SubToLeftSpec, requirements);

        RedSpec_LeftSpecToObs = new ActionCommand(redSpec_LeftSpecToObs, requirements);

        RedSpec_LeftSpecToMidWay = new ActionCommand(redSpec_LeftSpecToMidWay, requirements);

        RedSpec_MidSpecToObs = new ActionCommand(redSpec_MidSpecToObs, requirements);

        RedSpec_ObsToMidSpec = new ActionCommand(redSpec_ObsToMidSpec, requirements);

        RedSpec_ObsToRightSpec = new ActionCommand(redSpec_ObsToRightSpec, requirements);

        RedSpec_SpecDepoToObs = new ActionCommand(redSpec_SpecDepoToObs, requirements);

//        RedSpec_RightSpecToObs = new ActionCommand(redSpec_RightSpecToObs, requirements);
        RedSpec_RightSpecToSub = new ActionCommand(redSpec_RightSpecToSub, requirements);

        RedSpec_ObsToSub = new ActionCommand(redSpec_ObsToSub, requirements);

        RedSpec_SubToObs = new ActionCommand(redSpec_SubToObs, requirements);

        RedSpec_ObsSpecCheck = new ActionCommand(redSpec_ObsSpecCheck, requirements);

        switch (desiredState) {
            case "redSpec_StartToSub":
                addCommands(
                        new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(gripper::close),
                                new InstantCommand(wrist::specimen),
                                new InstantCommand(arm::specimen),
                                RedSpec_StartToSub

                        ),    new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH - 1030, 50),
                        new InstantCommand(gripper::open),
                                new WaitCommand(300)
                        )

                );

                break;



            case "redSpec_SubToLeftSpec":
                addCommands(
                        new ParallelCommandGroup(
                                RedSpec_SubToLeftSpec, new SequentialCommandGroup(
                                        new WaitCommand(200),
                                new DepositToStateCommand(arm, wrist, gripper, lift, "specimenToWall"),
                                new InstantCommand(() -> DepositState = "wall"))
                        )
                );
                break;

            case "RightSpecPickUpSpecimen":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        RedSpec_ObsToRightSpec,
                                        new LiftToStateCommand(lift, BotPositions.LIFT_WALL, BotPositions.LIFT_TOLERANCE)
                                ),
                                new InstantCommand(gripper::close),
                                new LiftToStateCommand(lift, BotPositions.LIFT_TRANSIT, BotPositions.LIFT_TOLERANCE)
                        )
                );
                break;

            case "specDepoToObs":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        RedSpec_SpecDepoToObs,
                                        new InstantCommand(gripper::close),
                                        new InstantCommand(wrist::specimen),
                                        new InstantCommand(arm::specimen)                                ),
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

    public void setBasket() {
        depositCurrentState = "basket";
    }

    public void setIntake() {
        depositCurrentState = "intake";
    }

    public void setWall() {
        depositCurrentState = "wall";
    }
}