package org.firstinspires.ftc.teamcode.Echo.Commands;

//RedSpec

import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCBasketAutoTraj.redBasket_BasketToAscentPark;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCBasketAutoTraj.redBasket_BasketToLeftSample;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCBasketAutoTraj.redBasket_BasketToMidSample;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCBasketAutoTraj.redBasket_BasketToRightSample;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCBasketAutoTraj.redBasket_BasketToSub1A;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCBasketAutoTraj.redBasket_BasketToSub1A_2;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCBasketAutoTraj.redBasket_BasketToSub1B;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCBasketAutoTraj.redBasket_BasketToSub1B_2;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCBasketAutoTraj.redBasket_BasketToSub2A;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCBasketAutoTraj.redBasket_BasketToSub2A_2;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCBasketAutoTraj.redBasket_BasketToSub2B;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCBasketAutoTraj.redBasket_BasketToSub2B_2;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCBasketAutoTraj.redBasket_BasketToSub3A;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCBasketAutoTraj.redBasket_BasketToSub3A_2;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCBasketAutoTraj.redBasket_BasketToSub3B;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCBasketAutoTraj.redBasket_BasketToSub3B_2;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCBasketAutoTraj.redBasket_LeftSampleToBasket;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCBasketAutoTraj.redBasket_MidSampleToBasket;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCBasketAutoTraj.redBasket_RightSampleToBasket;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCBasketAutoTraj.redBasket_StartToBasket;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCBasketAutoTraj.redBasket_StartToSub;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCBasketAutoTraj.redBasket_SubToBasket;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCBasketAutoTraj.redBasket_SubToBasket_2;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCBasketAutoTraj.redBasket_SubToSubIntakeA;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCBasketAutoTraj.redBasket_SubToWaypoint;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCSpecimenAutoTraj.redSpecEx_LeftDepoToMidSpec;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCSpecimenAutoTraj.redSpecEx_LeftSpecToLeftDepo;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCSpecimenAutoTraj.redSpecEx_MidDepoToRightSpec;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCSpecimenAutoTraj.redSpecEx_MidSpecToMidDepo;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCSpecimenAutoTraj.redSpecEx_ObsPrepToObsSpec;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCSpecimenAutoTraj.redSpecEx_RightDepoToObsPrep;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCSpecimenAutoTraj.redSpecEx_RightSpecToRightDepo;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCSpecimenAutoTraj.redSpecEx_SubToLeftSpecZone;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCSpecimenAutoTraj.redSpec_ObsToSub1;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCSpecimenAutoTraj.redSpec_ObsToSub2;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCSpecimenAutoTraj.redSpec_ObsToSub3;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCSpecimenAutoTraj.redSpec_ObsToSub4;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCSpecimenAutoTraj.redSpec_StartToSub;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCSpecimenAutoTraj.redSpec_SubToMidPoint;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCSpecimenAutoTraj.redSpec_SubToObs;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCSpecimenAutoTraj.redSpec_SubToObs2;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCSpecimenAutoTraj.redSpec_SubToObs3;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCSpecimenAutoTraj.redSpec_SubToObs4;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.WaitCommand;


import org.firstinspires.ftc.teamcode.Echo.Commands.IntakeCommands.IntakeGetSampleCommand;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Extendo;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.VIntake;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.TestBed.ActionCommand;
import org.firstinspires.ftc.teamcode.TestBed.ExampleSubsystem;

import java.util.Set;


public class ParallelActionCommand extends ParallelCommandGroup {

    public String depositCurrentState = "";

    //RedSpecimen
    private ActionCommand RedSpec_StartToSub;
    private ActionCommand RedSpec_SubToMidPoint;
    private ActionCommand RedSpec_ObsToSub1;
    private ActionCommand RedSpec_ObsToSub2;
    private ActionCommand RedSpec_ObsToSub3;
    private ActionCommand RedSpec_ObsToSub4;
    private ActionCommand RedSpec_SubToObs;
    private ActionCommand RedSpec_SubToObs2;
    public ActionCommand RedSpec_SubToObs3;
    public ActionCommand RedSpec_SubToObs4;
    private ActionCommand RedSpecEx_SubToLeftSpecZone;
    private ActionCommand RedSpecEx_LeftSpecToLeftDepo;
    private ActionCommand RedSpecEx_LeftDepoToMidSpec;
    private ActionCommand RedSpecEx_MidSpecToMidDepo;
    private ActionCommand RedSpecEx_MidDepoToRightSpec;
    private ActionCommand RedSpecEx_RightSpecToRightDepo;
    private ActionCommand RedSpecEx_ObsPrepToObsSpec;


    //RedBasket
    private ActionCommand RedBasket_StartToSub;
    private ActionCommand RedBasket_StartToBasket;
    private ActionCommand RedBasket_BasketToRightSample;
    private ActionCommand RedBasket_RightSampleToBasket;
    //    private ActionCommand RedBasket_RightSampleIntake;
    private ActionCommand RedBasket_BasketToMidSample;
    private ActionCommand RedBasket_MidSampleToBasket;
    //    private ActionCommand RedBasket_MidSampleIntake;
    private ActionCommand RedBasket_BasketToLeftSample;
    private ActionCommand RedBasket_LeftSampleToBasket;
    //    private ActionCommand RedBasket_LeftSampleIntake;
    private ActionCommand RedBasket_BasketToAscentPark;
    private ActionCommand RedBasket_BasketToSub1A;
    private ActionCommand RedBasket_BasketToSub1B;
    private ActionCommand RedBasket_BasketToSub2A;
    private ActionCommand RedBasket_BasketToSub2B;
    private ActionCommand RedBasket_BasketToSub3A;
    private ActionCommand RedBasket_BasketToSub3B;

    private ActionCommand RedBasket_BasketToSub1A_2;
    private ActionCommand RedBasket_BasketToSub1B_2;
    private ActionCommand RedBasket_BasketToSub2A_2;
    private ActionCommand RedBasket_BasketToSub2B_2;
    private ActionCommand RedBasket_BasketToSub3A_2;
    private ActionCommand RedBasket_BasketToSub3B_2;

    private ActionCommand RedBasket_BasketToSub4A;
    private ActionCommand RedBasket_BasketToSub4B;

    private ActionCommand RedBasket_SubToBasket;
    private ActionCommand RedBasket_SubToBasket_2;
    private ActionCommand RedBasket_SubToSubIntake;
    private ActionCommand RedBasket_SubToWaypoint;



    static String DepositState;


    public ParallelActionCommand(Arm arm, Wrist wrist, Gripper gripper, Lift lift, Extendo extendo, VIntake vintake, ExampleSubsystem exampleSubsystem, String desiredState/*, String setState*/) {
        Set<Subsystem> requirements = Set.of(exampleSubsystem);

        //RedSpecimen

        RedSpec_StartToSub = new ActionCommand(redSpec_StartToSub, requirements);

        RedSpec_SubToMidPoint = new ActionCommand(redSpec_SubToMidPoint, requirements);

        RedSpec_ObsToSub1 = new ActionCommand(redSpec_ObsToSub1, requirements);

        RedSpec_ObsToSub2 = new ActionCommand(redSpec_ObsToSub2, requirements);

        RedSpec_ObsToSub3 = new ActionCommand(redSpec_ObsToSub3, requirements);

        RedSpec_ObsToSub4 = new ActionCommand(redSpec_ObsToSub4, requirements);

        RedSpec_SubToObs = new ActionCommand(redSpec_SubToObs, requirements);

        RedSpec_SubToObs2 = new ActionCommand(redSpec_SubToObs2, requirements);

        RedSpec_SubToObs3 = new ActionCommand(redSpec_SubToObs3, requirements);

        RedSpec_SubToObs4 = new ActionCommand(redSpec_SubToObs4, requirements);

        RedSpecEx_LeftSpecToLeftDepo = new ActionCommand(redSpecEx_LeftSpecToLeftDepo, requirements);

        RedSpecEx_LeftDepoToMidSpec = new ActionCommand(redSpecEx_LeftDepoToMidSpec, requirements);

        RedSpecEx_MidSpecToMidDepo = new ActionCommand(redSpecEx_MidSpecToMidDepo, requirements);

        RedSpecEx_MidDepoToRightSpec = new ActionCommand(redSpecEx_MidDepoToRightSpec, requirements);

        RedSpecEx_RightSpecToRightDepo = new ActionCommand(redSpecEx_RightSpecToRightDepo, requirements);

        RedSpecEx_ObsPrepToObsSpec = new ActionCommand(redSpecEx_ObsPrepToObsSpec, requirements);


        //RedBasket

        RedBasket_StartToSub = new ActionCommand(redBasket_StartToSub, requirements);

        RedBasket_StartToBasket = new ActionCommand(redBasket_StartToBasket, requirements);

        RedBasket_BasketToRightSample = new ActionCommand(redBasket_BasketToRightSample, requirements);

        RedBasket_RightSampleToBasket = new ActionCommand(redBasket_RightSampleToBasket, requirements);

//        RedBasket_RightSampleIntake = new ActionCommand(redBasket_RightSampleIntake, requirements);

        RedBasket_BasketToMidSample = new ActionCommand(redBasket_BasketToMidSample, requirements);

        RedBasket_MidSampleToBasket = new ActionCommand(redBasket_MidSampleToBasket, requirements);

//        RedBasket_MidSampleIntake = new ActionCommand(redBasket_MidSampleIntake, requirements);

        RedBasket_BasketToLeftSample = new ActionCommand(redBasket_BasketToLeftSample, requirements);

        RedBasket_LeftSampleToBasket = new ActionCommand(redBasket_LeftSampleToBasket, requirements);

//        RedBasket_LeftSampleIntake = new ActionCommand(redBasket_LeftSampleIntake, requirements);

        RedBasket_BasketToAscentPark = new ActionCommand(redBasket_BasketToAscentPark, requirements);
        RedBasket_BasketToSub1A = new ActionCommand(redBasket_BasketToSub1A, requirements);
        RedBasket_BasketToSub1B = new ActionCommand(redBasket_BasketToSub1B, requirements);
        RedBasket_BasketToSub2A = new ActionCommand(redBasket_BasketToSub2A, requirements);
        RedBasket_BasketToSub2B = new ActionCommand(redBasket_BasketToSub2B, requirements);
        RedBasket_BasketToSub3A = new ActionCommand(redBasket_BasketToSub3A, requirements);
        RedBasket_BasketToSub3B = new ActionCommand(redBasket_BasketToSub3B, requirements);
        RedBasket_BasketToAscentPark = new ActionCommand(redBasket_BasketToAscentPark, requirements);
        RedBasket_BasketToSub1A_2 = new ActionCommand(redBasket_BasketToSub1A_2, requirements);
        RedBasket_BasketToSub1B_2 = new ActionCommand(redBasket_BasketToSub1B_2, requirements);
        RedBasket_BasketToSub2A_2 = new ActionCommand(redBasket_BasketToSub2A_2, requirements);
        RedBasket_BasketToSub2B_2 = new ActionCommand(redBasket_BasketToSub2B_2, requirements);
        RedBasket_BasketToSub3A_2 = new ActionCommand(redBasket_BasketToSub3A_2, requirements);
        RedBasket_BasketToSub3B_2 = new ActionCommand(redBasket_BasketToSub3B_2, requirements);
        RedBasket_SubToBasket = new ActionCommand(redBasket_SubToBasket, requirements);
        RedBasket_SubToBasket_2 = new ActionCommand(redBasket_SubToBasket_2, requirements);
        RedBasket_SubToSubIntake = new ActionCommand(redBasket_SubToSubIntakeA, requirements);
        RedBasket_SubToWaypoint = new ActionCommand(redBasket_SubToWaypoint, requirements);

        switch (desiredState) {

//RedSpecimen Commands

            case "redSpec_StartToSub":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new InstantCommand(gripper::close),
                                        new InstantCommand(wrist::specimen),
                                        new InstantCommand(arm::specimenAuto),
                                        new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH_AUTO, BotPositions.LIFT_TOLERANCE),
                                        RedSpec_StartToSub,
                                        new SequentialCommandGroup(
                                                new WaitCommand(1800),
                                                new InstantCommand(arm::specimenHangAuto),
                                                new WaitCommand(0),
                                                new InstantCommand(gripper::open)
                                        )
                                )


                        )
                );

                break;

            case "redSpec_SubToLeftSpec":
                addCommands(
                        new ParallelCommandGroup(
                                RedSpec_SubToMidPoint,
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new InstantCommand(() -> DepositState = "wall")
                                )
                        )
                );
                break;

            case "redSpec_RightSpecDepoToObs":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new ActionCommand(redSpecEx_RightDepoToObsPrep, requirements),
                                        new InstantCommand(gripper::open),
                                        new InstantCommand(extendo::in),
                                        new InstantCommand(vintake::transferPosition)
                                ),
                                new ParallelCommandGroup(
                                        new ActionCommand(redSpecEx_ObsPrepToObsSpec, requirements)
                                ),
                                new InstantCommand(gripper::close),
                                new WaitCommand(100)
                        )
                );
                break;

            case "redSpec_ObsToSub1":
                addCommands(
                        new SequentialCommandGroup(
                                new InstantCommand(gripper::close),
                                new ParallelCommandGroup(
                                        new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH_AUTO, BotPositions.LIFT_TOLERANCE),
                                        new SequentialCommandGroup(
                                                new WaitCommand(100),
                                                RedSpec_ObsToSub1),
                                        new SequentialCommandGroup(
                                                new WaitCommand(650),
                                                new InstantCommand(gripper::close),
                                                new InstantCommand(wrist::specimen),
                                                new InstantCommand(arm::specimenAuto)),
                                        new SequentialCommandGroup(
                                                new WaitCommand(2000),
                                                new InstantCommand(arm::specimenHangAuto),
                                                new WaitCommand(0),
                                                new InstantCommand(gripper::open)
                                        )
                                )
                        )
                );
                break;

            case "redSpec_ObsToSub2":
                addCommands(

                        new SequentialCommandGroup(
                                new InstantCommand(gripper::close),
                                new ParallelCommandGroup(
                                        new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH_AUTO, BotPositions.LIFT_TOLERANCE),
                                        new SequentialCommandGroup(
                                                new WaitCommand(100),
                                                RedSpec_ObsToSub2),
                                        new SequentialCommandGroup(
                                                new WaitCommand(650),
                                                new InstantCommand(gripper::close),
                                                new InstantCommand(wrist::specimen),
                                                new InstantCommand(arm::specimenAuto)),
                                        new SequentialCommandGroup(
                                                new WaitCommand(2000),
                                                new InstantCommand(arm::specimenHangAuto),
                                                new WaitCommand(0),
                                                new InstantCommand(gripper::open)
                                        )
                                )
                        )
                );
                break;

            case "redSpec_ObsToSub3":
                addCommands(

                        new SequentialCommandGroup(
                                new InstantCommand(gripper::close),
                                new ParallelCommandGroup(
                                        new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH_AUTO, BotPositions.LIFT_TOLERANCE),
                                        new SequentialCommandGroup(
                                                new WaitCommand(100),
                                                RedSpec_ObsToSub3),
                                        new SequentialCommandGroup(
                                                new WaitCommand(650),
                                                new InstantCommand(gripper::close),
                                                new InstantCommand(wrist::specimen),
                                                new InstantCommand(arm::specimenAuto)),
                                        new SequentialCommandGroup(
                                                new WaitCommand(2000),
                                                new InstantCommand(arm::specimenHangAuto),
                                                new WaitCommand(0),
                                                new InstantCommand(gripper::open)
                                        )
                                )
                        )
                );
                break;

            case "redSpec_ObsToSub4":
                addCommands(

                        new SequentialCommandGroup(
                                new InstantCommand(gripper::close),
                                new ParallelCommandGroup(
                                        new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH_AUTO, BotPositions.LIFT_TOLERANCE),
                                                new SequentialCommandGroup(
                                                new WaitCommand(100),
                                                RedSpec_ObsToSub4),
                                        new SequentialCommandGroup(
                                                new WaitCommand(650),
                                                new InstantCommand(gripper::close),
                                                new InstantCommand(wrist::specimen),
                                                new InstantCommand(arm::specimenAuto)),
                                        new SequentialCommandGroup(
                                                new WaitCommand(2000),
                                                new InstantCommand(arm::specimenHangAuto),
                                                new WaitCommand(0),
                                                new InstantCommand(gripper::open)
                                        )
                                )
                        )
                );
                break;


            case "redSpec_SubToObs":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        RedSpec_SubToObs,
                                        new SequentialCommandGroup(
                                                new WaitCommand(400),
                                                new DepositToStateAutoCommand(arm, wrist, gripper, lift, "specimenToWall")
                                        )
//                                        new SequentialCommandGroup(
//                                                new WaitCommand(1000),
//                                                new InstantCommand(gripper::open)
//
//                                        )
                                ),
                                new InstantCommand(gripper::close)
                        )
                );
                break;

            case "redSpec_SubToObs2":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        RedSpec_SubToObs2,
                                        new SequentialCommandGroup(
                                                new WaitCommand(400),
                                                new DepositToStateAutoCommand(arm, wrist, gripper, lift, "specimenToWall"))))
                );
                break;

            case "redSpec_SubToObs3":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        RedSpec_SubToObs3,
                                        new SequentialCommandGroup(
                                                new WaitCommand(400),
                                                new DepositToStateAutoCommand(arm, wrist, gripper, lift, "specimenToWall"))))
                );
                break;

            case "redSpec_SubToObs4":
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                RedSpec_SubToObs4,
                                new SequentialCommandGroup(
                                        new WaitCommand(400),
                                        new DepositToStateAutoCommand(arm, wrist, gripper, lift, "specimenToWall")))
                );
                break;

            case "redSpec_Park":
                addCommands(
                        new SequentialCommandGroup(
                                RedSpec_SubToObs
                        )
                );
                break;


            case "redSpecEx_LeftSpecDepo":
                addCommands(
                        new ParallelCommandGroup(
                                new DepositToStateAutoCommand(arm, wrist, gripper, lift, "specimenToWall"),
                                new SequentialCommandGroup(
                                        new ActionCommand(redSpecEx_SubToLeftSpecZone, requirements), // why an action command
//                                        new WaitCommand(300),
                                        RedSpecEx_LeftSpecToLeftDepo
                                ),
                                new SequentialCommandGroup(
                                        new WaitCommand(1000),
                                        new InstantCommand(extendo::outAuto),
                                        new InstantCommand(extendo::outAuto),//to avoid the extendo stucking
                                        new InstantCommand(vintake::sweepPosition))
                        ))

                ;
                break;

            case "redSpecEx_MidSpecDepo":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new InstantCommand(vintake::upPosition),
                                        RedSpecEx_LeftDepoToMidSpec),
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new InstantCommand(vintake::sweepPosition),
                                                new SequentialCommandGroup(
                                                        //new WaitCommand(300),
                                                        RedSpecEx_MidSpecToMidDepo))
                                ))
                );
                break;

            case "redSpecEx_RightSpecDepo":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new InstantCommand(vintake::upPosition),
                                        RedSpecEx_MidDepoToRightSpec),
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new InstantCommand(vintake::sweepPosition),
                                                new SequentialCommandGroup(
                                                        //new WaitCommand(100),
                                                        RedSpecEx_RightSpecToRightDepo))
                                )
                        ));
                break;


//RedBasket Commands

            case "redBasket_StartToBasketDepo":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new WaitCommand(500),
                                                RedBasket_StartToBasket),
                                        new InstantCommand(gripper::close),
                                        new DepositToStateAutoCommand(arm, wrist, gripper, lift, "intakeToBasketHigh")
                                ), new InstantCommand(extendo::outAuto),
                                new WaitCommand(100),
                                new InstantCommand(gripper::open)
                        )
                );

                break;

            case "redBasket_StartToSub":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new InstantCommand(gripper::close),
                                        new InstantCommand(wrist::specimen),
                                        new InstantCommand(arm::specimenAuto),
                                        new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH_AUTO, BotPositions.LIFT_TOLERANCE),
                                        RedBasket_StartToSub,
                                        new SequentialCommandGroup(
                                                new WaitCommand(1800),
                                                new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH, 70),
                                                new WaitCommand(100),
                                                new InstantCommand(gripper::open)
                                        )
                                )


                        )
                );
                break;

            case "redBasket_IntakeRightSample":
                addCommands(
                        new SequentialCommandGroup(

                                //aware if the gripper will hit the basket
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new WaitCommand(200),
                                                RedBasket_BasketToRightSample,
                                                new InstantCommand(vintake::downPosition)
                                                ),


                                        new InstantCommand(vintake::in),
                                        new SequentialCommandGroup(
                                                new WaitCommand(800),
                                                new DepositToStateAutoCommand(arm, wrist, gripper, lift, "basketToIntake")
                                        )),

                                new ParallelCommandGroup(
                                        new IntakeGetSampleCommand(vintake)))

                        //   new IntakeGetSampleCommand(intake))//runs until sample is acquired

                );
                break;

            case "redBasket_ScoreRightSample":
                addCommands(
                        new SequentialCommandGroup(
                                new SequentialCommandGroup(
                                        new InstantCommand(vintake::transferPosition),

                                        new InstantCommand(extendo::in)

                                ),
                                new SequentialCommandGroup(
//                                        new SequentialCommandGroup(
//                                                new WaitCommand(250),
//                                                new InstantCommand(intake::transfer)),
                                        new GripperAutoCloseCommand(gripper),
                                        new WaitCommand(400),
                                        new InstantCommand(vintake::stop),
                                        new ParallelCommandGroup(
                                                new DepositToStateAutoCommand(arm, wrist, gripper, lift, "intakeToBasketHigh"),
                                                new SequentialCommandGroup(
                                                        new WaitCommand(400),//500
                                                        new WaitCommand(400),
                                                        RedBasket_RightSampleToBasket)),

                                        new InstantCommand(gripper::open),
                                        new InstantCommand(extendo::outAuto)
                                )
                        ));
                break;

            case "redBasket_IntakeMidSample":
                addCommands(
                        new SequentialCommandGroup(
                                //aware if the gripper will hit the basket
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new WaitCommand(200),
                                                RedBasket_BasketToMidSample,
                                                new InstantCommand(vintake::downPosition)),


                                        new InstantCommand(vintake::in),
                                        new SequentialCommandGroup(
                                                new WaitCommand(1500),
                                                new DepositToStateAutoCommand(arm, wrist, gripper, lift, "basketToIntake")
                                        ),
                                        new ParallelCommandGroup(
                                                new IntakeGetSampleCommand(vintake)))


                                //   new IntakeGetSampleCommand(intake))//runs until sample is acquired
                        )
                );
                break;

            case "redBasket_ScoreMidSample":
                addCommands(
                        new SequentialCommandGroup(
                                new SequentialCommandGroup(
                                        new InstantCommand(vintake::transferPosition),

                                        new InstantCommand(extendo::in)

                                ),
                                new SequentialCommandGroup(
//                                        new SequentialCommandGroup(
//                                                new WaitCommand(250),
//                                                new InstantCommand(intake::transfer)),
                                        new GripperAutoCloseCommand(gripper),
                                        new WaitCommand(400),
                                        new InstantCommand(vintake::stop),
                                        new ParallelCommandGroup(
                                                new DepositToStateAutoCommand(arm, wrist, gripper, lift, "intakeToBasketHigh"),
                                                new SequentialCommandGroup(
                                                        new WaitCommand(400),//500

                                                        new WaitCommand(400),
                                                        RedBasket_MidSampleToBasket)),
                                        new WaitCommand(100),
                                        new InstantCommand(gripper::open),
                                        new InstantCommand(extendo::outAuto)
                                )
                        ));
                break;

            case "redBasket_IntakeLeftSample":
                addCommands(
                        new SequentialCommandGroup(
                                //aware if the gripper will hit the basket
                                //aware if the gripper will hit the basket
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new WaitCommand(200),
                                                RedBasket_BasketToLeftSample,
                                                new InstantCommand(vintake::downPosition)),


                                        new InstantCommand(vintake::in),

                                        new SequentialCommandGroup(
                                                new WaitCommand(1500),
                                                new DepositToStateAutoCommand(arm, wrist, gripper, lift, "basketToIntake")
                                        ), new ParallelCommandGroup(
                                        new IntakeGetSampleCommand(vintake)))

                                //   new IntakeGetSampleCommand(intake))//runs until sample is acquired
                        )
                );
                break;

            case "redBasket_ScoreLeftSample":
                addCommands(
                        new SequentialCommandGroup(
                                new SequentialCommandGroup(
                                        new InstantCommand(vintake::transferPosition),

                                        new InstantCommand(extendo::in)

                                ),
                                new SequentialCommandGroup(
//                                        new SequentialCommandGroup(
//                                                new WaitCommand(250),
//                                                new InstantCommand(intake::transfer)),
                                        new GripperAutoCloseCommand(gripper),
                                        new WaitCommand(400),
                                        new InstantCommand(vintake::stop),
                                        new ParallelCommandGroup(
                                                new DepositToStateAutoCommand(arm, wrist, gripper, lift, "intakeToBasketHigh"),
                                                new SequentialCommandGroup(
                                                        new WaitCommand(400),//500

                                                        new WaitCommand(400),
                                                        RedBasket_LeftSampleToBasket)),
                                        new WaitCommand(100),
                                        new InstantCommand(gripper::open)
                                )
                        ));
                break;

            case "redBasket_BasketToSub1A":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new WaitCommand(1000),
                                                new DepositToStateAutoCommand(arm, wrist, gripper, lift, "basketToIntake")),
                                        RedBasket_BasketToSub1A),
                                new InstantCommand(extendo::outAuto),
                                new InstantCommand(vintake::in),
                                new WaitCommand(300),
                                new InstantCommand(vintake::downPosition),
                                new IntakeGetSampleCommand(vintake),
                                new InstantCommand(extendo::in),
                                new InstantCommand(vintake::transferPosition),
                                new WaitCommand(1000),
                                new GripperAutoCloseCommand(gripper),
                new InstantCommand(gripper::close),
                new WaitCommand(300),
                new InstantCommand(vintake::stop)
                        ));
                break;
            case "redBasket_BasketToSub2A":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new WaitCommand(1000),
                                                new DepositToStateAutoCommand(arm, wrist, gripper, lift, "basketToIntake")),
                                        RedBasket_BasketToSub2A),
                                new InstantCommand(extendo::outAuto),
                                new InstantCommand(vintake::in),
                                new WaitCommand(300),
                                new InstantCommand(vintake::downPosition),
                                new IntakeGetSampleCommand(vintake),
                                new InstantCommand(extendo::in),
                                new InstantCommand(vintake::transferPosition),
                                new WaitCommand(1000),
                                new GripperAutoCloseCommand(gripper),
                                new InstantCommand(gripper::close),
                                new WaitCommand(300),
                                new InstantCommand(vintake::stop)
                        ));
                break;
            case "redBasket_BasketToSub3A":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new WaitCommand(1000),
                                                new DepositToStateAutoCommand(arm, wrist, gripper, lift, "basketToIntake")),
                                        RedBasket_BasketToSub3A),
                                new InstantCommand(extendo::outAuto),
                                new InstantCommand(vintake::in),
                                new WaitCommand(300),
                                new InstantCommand(vintake::downPosition),
                                new IntakeGetSampleCommand(vintake),
                                new InstantCommand(extendo::in),
                                new InstantCommand(vintake::transferPosition),
                                new WaitCommand(1000),
                                new GripperAutoCloseCommand(gripper),
                                new InstantCommand(gripper::close),
                                new WaitCommand(300),
                                new InstantCommand(vintake::stop)
                        ));
                break;
            case "redBasket_BasketToSub1B":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new WaitCommand(1000),
                                                new DepositToStateAutoCommand(arm, wrist, gripper, lift, "basketToIntake")),
                                        RedBasket_BasketToSub1B),
                                new InstantCommand(extendo::outAuto),
                                new InstantCommand(vintake::in),
                                new WaitCommand(300),
                                new InstantCommand(vintake::downPosition),
                                new IntakeGetSampleCommand(vintake),
                                new InstantCommand(extendo::in),
                                new InstantCommand(vintake::transferPosition),
                                new WaitCommand(1000),
                                new GripperAutoCloseCommand(gripper),
                                new InstantCommand(gripper::close),
                                new WaitCommand(300),
                                new InstantCommand(vintake::stop)
                        ));
                break;
            case "redBasket_BasketToSub2B":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new WaitCommand(1000),
                                                new DepositToStateAutoCommand(arm, wrist, gripper, lift, "basketToIntake")),
                                        RedBasket_BasketToSub2B),
                                new InstantCommand(extendo::outAuto),
                                new InstantCommand(vintake::in),
                                new WaitCommand(300),
                                new InstantCommand(vintake::downPosition),
                                new IntakeGetSampleCommand(vintake),
                                new InstantCommand(extendo::in),
                                new InstantCommand(vintake::transferPosition),
                                new WaitCommand(1000),
                                new GripperAutoCloseCommand(gripper),
                                new InstantCommand(gripper::close),
                                new WaitCommand(300),
                                new InstantCommand(vintake::stop)
                        ));
                break;
            case "redBasket_BasketToSub3B":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new WaitCommand(1000),
                                                new DepositToStateAutoCommand(arm, wrist, gripper, lift, "basketToIntake")),
                                        RedBasket_BasketToSub3B),
                                new InstantCommand(extendo::outAuto),
                                new InstantCommand(vintake::in),
                                new WaitCommand(300),
                                new InstantCommand(vintake::downPosition),
                                new IntakeGetSampleCommand(vintake),
                                new InstantCommand(extendo::in),
                                new InstantCommand(vintake::transferPosition),
                                new WaitCommand(1000),
                                new GripperAutoCloseCommand(gripper),
                                new InstantCommand(gripper::close),
                                new WaitCommand(300),
                                new InstantCommand(vintake::stop)
                        ));
                break;

            case "redBasket_BasketToSub1A_2":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new WaitCommand(1000),
                                                new DepositToStateAutoCommand(arm, wrist, gripper, lift, "basketToIntake")),
                                        RedBasket_BasketToSub1A_2),
                                new InstantCommand(extendo::outAuto),
                                new InstantCommand(vintake::in),
                                new WaitCommand(300),
                                new InstantCommand(vintake::downPosition),
                                new IntakeGetSampleCommand(vintake),
                                new InstantCommand(extendo::in),
                                new InstantCommand(vintake::transferPosition),
                                new WaitCommand(1000),
                                new GripperAutoCloseCommand(gripper),
                                new InstantCommand(gripper::close),
                                new WaitCommand(300),
                                new InstantCommand(vintake::stop)
                        ));
                break;
            case "redBasket_BasketToSub2A_2":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new WaitCommand(1000),
                                                new DepositToStateAutoCommand(arm, wrist, gripper, lift, "basketToIntake")),
                                        RedBasket_BasketToSub2A_2),
                                new InstantCommand(extendo::outAuto),
                                new InstantCommand(vintake::in),
                                new WaitCommand(300),
                                new InstantCommand(vintake::downPosition),
                                new IntakeGetSampleCommand(vintake),
                                new InstantCommand(extendo::in),
                                new InstantCommand(vintake::transferPosition),
                                new WaitCommand(1000),
                                new GripperAutoCloseCommand(gripper),
                                new InstantCommand(gripper::close),
                                new WaitCommand(300),
                                new InstantCommand(vintake::stop)
                        ));
                break;
            case "redBasket_BasketToSub3A_2":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new WaitCommand(1000),
                                                new DepositToStateAutoCommand(arm, wrist, gripper, lift, "basketToIntake")),
                                        RedBasket_BasketToSub3A_2),
                                new InstantCommand(extendo::outAuto),
                                new InstantCommand(vintake::in),
                                new WaitCommand(300),
                                new InstantCommand(vintake::downPosition),
                                new IntakeGetSampleCommand(vintake),
                                new InstantCommand(extendo::in),
                                new InstantCommand(vintake::transferPosition),
                                new WaitCommand(1000),
                                new GripperAutoCloseCommand(gripper),
                                new InstantCommand(gripper::close),
                                new WaitCommand(300),
                                new InstantCommand(vintake::stop)
                        ));
                break;
            case "redBasket_BasketToSub1B_2":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new WaitCommand(1000),
                                                new DepositToStateAutoCommand(arm, wrist, gripper, lift, "basketToIntake")),
                                        RedBasket_BasketToSub1B_2),
                                new InstantCommand(extendo::outAuto),
                                new InstantCommand(vintake::in),
                                new WaitCommand(300),
                                new InstantCommand(vintake::downPosition),
                                new IntakeGetSampleCommand(vintake),
                                new InstantCommand(extendo::in),
                                new InstantCommand(vintake::transferPosition),
                                new WaitCommand(1000),
                                new GripperAutoCloseCommand(gripper),
                                new InstantCommand(gripper::close),
                                new WaitCommand(300),
                                new InstantCommand(vintake::stop)
                        ));
                break;
            case "redBasket_BasketToSub2B_2":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new WaitCommand(1500),
                                                new DepositToStateAutoCommand(arm, wrist, gripper, lift, "basketToIntake")),
                                        RedBasket_BasketToSub2B_2),
                                new InstantCommand(extendo::outAuto),
                                new InstantCommand(vintake::in),
                                new WaitCommand(300),
                                new InstantCommand(vintake::downPosition),
                                new IntakeGetSampleCommand(vintake),
                                new InstantCommand(extendo::in),
                                new InstantCommand(vintake::transferPosition),
                                new WaitCommand(1000),
                                new GripperAutoCloseCommand(gripper),
                                new InstantCommand(gripper::close),
                                new WaitCommand(300),
                                new InstantCommand(vintake::stop)
                        ));
                break;
            case "redBasket_BasketToSub3B_2":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new WaitCommand(1000),
                                                new DepositToStateAutoCommand(arm, wrist, gripper, lift, "basketToIntake")),
                                        RedBasket_BasketToSub3B_2),
                                new InstantCommand(extendo::outAuto),
                                new InstantCommand(vintake::in),
                                new WaitCommand(300),
                                new InstantCommand(vintake::downPosition),
                                new IntakeGetSampleCommand(vintake),
                                new InstantCommand(extendo::in),
                                new InstantCommand(vintake::transferPosition),
                                new WaitCommand(1000),
                                new GripperAutoCloseCommand(gripper),
                                new InstantCommand(gripper::close),
                                new WaitCommand(300),
                                new InstantCommand(vintake::stop)
                        ));
                break;



            case "redBasket_SubToBasket":
                addCommands(
                        new SequentialCommandGroup(
RedBasket_SubToWaypoint,
                                new ParallelCommandGroup(

                                        new DepositToStateAutoCommand(arm, wrist, gripper, lift, "intakeToBasketHigh"),
                                                RedBasket_SubToBasket),
                                new WaitCommand(400),
                                new InstantCommand(gripper::open)
                        ));
                break;
            case "redBasket_SubToBasket_2":
                addCommands(
                        new SequentialCommandGroup(
                                RedBasket_SubToWaypoint,
                                new ParallelCommandGroup(

                                        new DepositToStateAutoCommand(arm, wrist, gripper, lift, "intakeToBasketHigh"),
                                        RedBasket_SubToBasket_2),
                                new WaitCommand(400),
                                new InstantCommand(gripper::open)
                        ));
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