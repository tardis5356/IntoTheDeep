package org.firstinspires.ftc.teamcode.Echo.Commands;

//RedSpec
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_MidPointToLeftSpec;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_LeftSpecToObs;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_MidSpecToObs;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_ObsSpecCheck;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_ObsToMidSpec;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_ObsToRightSpec;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_ObsToSub;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_RightSpecToObs;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_RightSpecToSub;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_SpecDepoToObs;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_StartToSub;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_SubToMidPoint;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_SubToObs;

//RedBasket
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redBasket_StartToSub;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redBasket_SubToRightSample;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redBasket_RightSampleToBasket;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redBasket_BasketToMidSample;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redBasket_MidSampleToBasket;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redBasket_BasketToLeftSample;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redBasket_LeftSampleToBasket;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redBasket_BasketToAscentPark;


import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Echo.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.TestBed.ActionCommand;
import org.firstinspires.ftc.teamcode.TestBed.ExampleSubsystem;

import java.util.Set;


public class ParallelActionCommand extends ParallelCommandGroup {

    public String depositCurrentState = "";

   //RedSpecimen
    private ActionCommand RedSpec_StartToSub;
    private ActionCommand RedSpec_SubToMidPoint;
    private ActionCommand RedSpec_MidPointToLeftSpec;
    private ActionCommand RedSpec_LeftSpecToObs;
    private ActionCommand RedSpec_ObsToMidSpec;
    private ActionCommand RedSpec_MidSpecToObs;
    private ActionCommand RedSpec_ObsToRightSpec;

    private ActionCommand RedSpec_RightSpecToObs;
    private ActionCommand RedSpec_SpecDepoToObs;
    private ActionCommand RedSpec_ObsToSub;
    private ActionCommand RedSpec_SubToObs;


    //RedBasket
    private ActionCommand RedBasket_StartToSub;
    private ActionCommand RedBasket_SubToRightSample;
    private ActionCommand RedBasket_RightSampleToBasket;
    private ActionCommand RedBasket_BasketToMidSample;
    private ActionCommand RedBasket_MidSampleToBasket;
    private ActionCommand RedBasket_BasketToLeftSample;
    private ActionCommand RedBasket_LeftSampleToBasket;
    private ActionCommand RedBasket_BasketToAscentPark;


    static String DepositState;


    public ParallelActionCommand(Arm arm, Wrist wrist, Gripper gripper, Lift lift, ExampleSubsystem exampleSubsystem, String desiredState/*, String setState*/) {
        Set<Subsystem> requirements = Set.of(exampleSubsystem);

        //RedSpecimen
        RedSpec_StartToSub = new ActionCommand(redSpec_StartToSub, requirements);

        RedSpec_MidPointToLeftSpec = new ActionCommand(redSpec_MidPointToLeftSpec, requirements);

        RedSpec_SubToMidPoint = new ActionCommand(redSpec_SubToMidPoint,requirements);

        RedSpec_LeftSpecToObs = new ActionCommand(redSpec_LeftSpecToObs, requirements);

        RedSpec_MidSpecToObs = new ActionCommand(redSpec_MidSpecToObs, requirements);

        RedSpec_ObsToMidSpec = new ActionCommand(redSpec_ObsToMidSpec, requirements);

        RedSpec_ObsToRightSpec = new ActionCommand(redSpec_ObsToRightSpec, requirements);

        RedSpec_RightSpecToObs =  new ActionCommand(redSpec_RightSpecToObs, requirements);

        RedSpec_SpecDepoToObs = new ActionCommand(redSpec_SpecDepoToObs, requirements);

        RedSpec_ObsToSub = new ActionCommand(redSpec_ObsToSub, requirements);

        RedSpec_SubToObs = new ActionCommand(redSpec_SubToObs, requirements);


        //RedBasket
        RedBasket_StartToSub = new ActionCommand(redBasket_StartToSub, requirements);

        RedBasket_SubToRightSample = new ActionCommand(redBasket_SubToRightSample, requirements);

        RedBasket_RightSampleToBasket = new ActionCommand(redBasket_RightSampleToBasket, requirements);

        RedBasket_BasketToMidSample = new ActionCommand(redBasket_BasketToMidSample, requirements);

        RedBasket_MidSampleToBasket = new ActionCommand(redBasket_MidSampleToBasket, requirements);

        RedBasket_BasketToLeftSample = new ActionCommand(redBasket_BasketToLeftSample, requirements);

        RedBasket_LeftSampleToBasket = new ActionCommand(redBasket_LeftSampleToBasket,requirements);

        RedBasket_BasketToAscentPark = new ActionCommand(redBasket_BasketToAscentPark,requirements);


        switch (desiredState) {

            //redSpecimen

            case "redSpec_StartToSub":
                addCommands(
                        new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(gripper::close),
                                new InstantCommand(wrist::specimen),
                                new InstantCommand(arm::specimen),
                                RedSpec_StartToSub

                        ),
                                new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH - 1030, 50),
                        new InstantCommand(gripper::open),
                                new LiftToStateCommand(lift, BotPositions.LIFT_TRANSIT, BotPositions.LIFT_TOLERANCE),
                                new WaitCommand(500)
                        )

                );

                break;



            case "redSpec_SubToLeftSpec":
                addCommands(
                        new ParallelCommandGroup(
                                RedSpec_SubToMidPoint,
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                //new DepositToStateCommand(arm, wrist, gripper, lift, "specimenToWall"),
                                new InstantCommand(() -> DepositState = "wall")
                                )
                        )
                );
                break;

            case "redSpec_RightSpecPickUpSpecimen":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        RedSpec_RightSpecToObs,
                                        new LiftToStateCommand(lift, BotPositions.LIFT_WALL, BotPositions.LIFT_TOLERANCE),
                                        new InstantCommand(gripper::open)
                                ),
                                new WaitCommand(2000),
                                new InstantCommand(gripper::close)
                        )
                );
                break;

            case "redSpec_RightSpecDepoToSub":
                addCommands(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(gripper::close),
                                new InstantCommand(wrist::specimen),
                                new InstantCommand(arm::specimen),
                                RedSpec_RightSpecToObs

                        ),
                        new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH - 1030, 50),
                        new InstantCommand(gripper::open),
                        new WaitCommand(300)
                )
                );
                break;

//            case "specDepoToObs":
//                addCommands(
//                        new SequentialCommandGroup(
//                                new ParallelCommandGroup(
//                                        RedSpec_SpecDepoToObs,
//                                        new InstantCommand(gripper::close),
//                                        new InstantCommand(wrist::specimen),
//                                        new InstantCommand(arm::specimen)                                ),
//                                RedSpec_ObsSpecCheck,
//                                new WaitCommand(1500),
//                          new InstantCommand(gripper::close)
//                        )
//                );
//                break;

            case "redSpec_ObsToSub":
                addCommands(
                        new SequentialCommandGroup(
                                RedSpec_ObsToSub,
                                new DepositToStateCommand(arm, wrist, gripper, lift, "wallToSpecimen"),
                                new InstantCommand(() -> DepositState = "specimen")),
                        new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH - 1030, BotPositions.LIFT_TOLERANCE),
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

            case "redBasket_StartToSub":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new InstantCommand(gripper::close),
                                        new InstantCommand(wrist::specimen),
                                        new InstantCommand(arm::specimen),
                                        RedBasket_StartToSub

                                ),
                                new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH - 1030, 50),
                                new InstantCommand(gripper::open)
                        )

                );

                break;
    }}

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

