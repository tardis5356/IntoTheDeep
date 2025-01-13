package org.firstinspires.ftc.teamcode.Echo.Commands;

//RedSpec

import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpecEx_LeftDepoToMidSpec;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpecEx_LeftSpecToLeftDepo;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpecEx_MidDepoToRightSpec;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpecEx_MidSpecToMidDepo;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpecEx_ObsPrepToObsSpec;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpecEx_RightDepoToObsPrep;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpecEx_RightSpecToRightDepo;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpecEx_SubToLeftSpecZone;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_MidPointToLeftSpec;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_LeftSpecToObs;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_MidSpecToObs;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_ObsToMidSpec;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_ObsToRightSpec;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_ObsToSub;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_ObsToSub1;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_ObsToSub2;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_ObsToSub3;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_RightSpecToObs;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_RightSpecObsPickUpToSub;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_SpecDepoToObs;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_StartToSub;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_SubToMidPoint;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_SubToObs;

//RedBasket
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redBasket_StartToSub;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redBasket_StartToBasket;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redBasket_RightSampleToBasket;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redBasket_BasketToRightSample;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redBasket_RightSampleIntake;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redBasket_BasketToMidSample;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redBasket_MidSampleToBasket;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redBasket_MidSampleIntake;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redBasket_BasketToLeftSample;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redBasket_LeftSampleToBasket;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redBasket_LeftSampleIntake;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redBasket_BasketToAscentPark;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_SubToObs2;


import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Echo.Commands.IntakeCommands.IntakeGetSampleCommand;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Extendo;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Intake;
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

    private ActionCommand RedSpec_RightSpecObsPickUpToSub;
    private ActionCommand RedSpec_RightSpecToObs;
    private ActionCommand RedSpec_SpecDepoToObs;
    private ActionCommand RedSpec_ObsToSub;

    private ActionCommand RedSpec_ObsToSub2;

    private ActionCommand RedSpec_ObsToSub3;
    private ActionCommand RedSpec_SubToObs;
    private ActionCommand RedSpec_SubToObs2;

    private ActionCommand RedSpec_SubToObs3;

    private ActionCommand RedSpecEx_SubToLeftSpecZone;
    private ActionCommand RedSpecEx_LeftSpecToLeftDepo;
    private ActionCommand RedSpecEx_LeftDepoToMidSpec;
    private ActionCommand RedSpecEx_MidSpecToMidDepo;
    private ActionCommand RedSpecEx_MidDepoToRightSpec;
    private ActionCommand RedSpecEx_RightSpecToRightDepo;


    //RedBasket
    private ActionCommand RedBasket_StartToSub;
    private ActionCommand RedBasket_StartToBasket;
    private ActionCommand RedBasket_BasketToRightSample;
    private ActionCommand RedBasket_RightSampleToBasket;
    private ActionCommand RedBasket_RightSampleIntake;
    private ActionCommand RedBasket_BasketToMidSample;
    private ActionCommand RedBasket_MidSampleToBasket;
    private ActionCommand RedBasket_MidSampleIntake;
    private ActionCommand RedBasket_BasketToLeftSample;
    private ActionCommand RedBasket_LeftSampleToBasket;
    private ActionCommand RedBasket_LeftSampleIntake;
    private ActionCommand RedBasket_BasketToAscentPark;


    static String DepositState;


    public ParallelActionCommand(Arm arm, Wrist wrist, Gripper gripper, Lift lift, Extendo extendo, Intake intake, ExampleSubsystem exampleSubsystem, String desiredState/*, String setState*/) {
        Set<Subsystem> requirements = Set.of(exampleSubsystem);

        //RedSpecimen
        RedSpec_StartToSub = new ActionCommand(redSpec_StartToSub, requirements);

        RedSpec_MidPointToLeftSpec = new ActionCommand(redSpec_MidPointToLeftSpec, requirements);

        RedSpec_SubToMidPoint = new ActionCommand(redSpec_SubToMidPoint, requirements);

        RedSpec_LeftSpecToObs = new ActionCommand(redSpec_LeftSpecToObs, requirements);

        RedSpec_MidSpecToObs = new ActionCommand(redSpec_MidSpecToObs, requirements);

        RedSpec_ObsToMidSpec = new ActionCommand(redSpec_ObsToMidSpec, requirements);

        RedSpec_ObsToRightSpec = new ActionCommand(redSpec_ObsToRightSpec, requirements);

        RedSpec_RightSpecObsPickUpToSub = new ActionCommand(redSpec_RightSpecObsPickUpToSub, requirements);

        RedSpec_RightSpecToObs = new ActionCommand(redSpec_RightSpecToObs, requirements);

        RedSpec_SpecDepoToObs = new ActionCommand(redSpec_SpecDepoToObs, requirements);

        RedSpec_ObsToSub = new ActionCommand(redSpec_ObsToSub1, requirements);

        RedSpec_ObsToSub2 = new ActionCommand(redSpec_ObsToSub2, requirements);

        RedSpec_ObsToSub3 = new ActionCommand(redSpec_ObsToSub3, requirements);

        RedSpec_SubToObs = new ActionCommand(redSpec_SubToObs, requirements);

        RedSpec_SubToObs2 = new ActionCommand(redSpec_SubToObs2, requirements);


        RedSpecEx_LeftSpecToLeftDepo = new ActionCommand(redSpecEx_LeftSpecToLeftDepo, requirements);

        RedSpecEx_LeftDepoToMidSpec = new ActionCommand(redSpecEx_LeftDepoToMidSpec, requirements);

        RedSpecEx_MidSpecToMidDepo = new ActionCommand(redSpecEx_MidSpecToMidDepo, requirements);

        RedSpecEx_MidDepoToRightSpec = new ActionCommand(redSpecEx_MidDepoToRightSpec, requirements);

        RedSpecEx_RightSpecToRightDepo = new ActionCommand(redSpecEx_RightSpecToRightDepo, requirements);


        //RedBasket

        RedBasket_StartToSub = new ActionCommand(redBasket_StartToSub, requirements);

        RedBasket_StartToBasket = new ActionCommand(redBasket_StartToBasket, requirements);

        RedBasket_BasketToRightSample = new ActionCommand(redBasket_BasketToRightSample, requirements);

        RedBasket_RightSampleToBasket = new ActionCommand(redBasket_RightSampleToBasket, requirements);

        RedBasket_RightSampleIntake = new ActionCommand(redBasket_RightSampleIntake, requirements);

        RedBasket_BasketToMidSample = new ActionCommand(redBasket_BasketToMidSample, requirements);

        RedBasket_MidSampleToBasket = new ActionCommand(redBasket_MidSampleToBasket, requirements);

        RedBasket_MidSampleIntake = new ActionCommand(redBasket_MidSampleIntake, requirements);

        RedBasket_BasketToLeftSample = new ActionCommand(redBasket_BasketToLeftSample, requirements);

        RedBasket_LeftSampleToBasket = new ActionCommand(redBasket_LeftSampleToBasket, requirements);

        RedBasket_LeftSampleIntake = new ActionCommand(redBasket_LeftSampleIntake, requirements);

        RedBasket_BasketToAscentPark = new ActionCommand(redBasket_BasketToAscentPark, requirements);


        switch (desiredState) {

//RedSpecimen Commands

            case "redSpec_StartToSub":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new InstantCommand(gripper::close),
                                        new InstantCommand(wrist::specimen),
                                        new InstantCommand(arm::specimen),
                                        new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH, BotPositions.LIFT_TOLERANCE),
                                        RedSpec_StartToSub,
                                        new SequentialCommandGroup(
                                        new WaitCommand(2200),
                                        new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH_CLIP, 70),
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
                                        new InstantCommand(intake::transferPosition)
                                ),
                                new ParallelCommandGroup(
                                        new InstantCommand(gripper::open),
                                        new ActionCommand(redSpecEx_ObsPrepToObsSpec, requirements),
                                        new GripperAutoCloseCommand(gripper)
                                )
                        )
                );
                break;

            case "redSpec_ObsToSub1":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new DepositToStateCommand(arm, wrist, gripper, lift, "wallToSpecimen"),
                                        RedSpec_ObsToSub
                                ),
                                new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH_CLIP, 70),
                                new InstantCommand(gripper::open),
                                new WaitCommand(100)
                        )
                );
                break;

            case "redSpec_ObsToSub2":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new DepositToStateCommand(arm, wrist, gripper, lift, "wallToSpecimen"),
                                        RedSpec_ObsToSub2
                                ),
                                new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH_CLIP, 70),
                                new InstantCommand(gripper::open),
                                new WaitCommand(100)
                        )
                );
                break;

            case "redSpec_ObsToSub3":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new DepositToStateCommand(arm, wrist, gripper, lift, "wallToSpecimen"),
                                        RedSpec_ObsToSub3
                                ),
                                new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH_CLIP, 70),
                                new InstantCommand(gripper::open),
                                new WaitCommand(100)
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

            case "redSpec_SubToObs":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        RedSpec_SubToObs,
                                        new DepositToStateCommand(arm, wrist, gripper, lift, "specimenToWall"),
                                        new SequentialCommandGroup(
                                                new WaitCommand(1500),
                                                new InstantCommand(gripper::open)

                                        )
                                ),
                                new GripperAutoCloseCommand(gripper)
                        )
                );
                break;

            case "redSpec_SubToObs2":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        RedSpec_SubToObs2,
                                        new DepositToStateCommand(arm, wrist, gripper, lift, "specimenToWall"),
                                        new SequentialCommandGroup(
                                                new WaitCommand(1500),
                                                new InstantCommand(gripper::open)
                                        )
                                ),
                                new GripperAutoCloseCommand(gripper),
                                new WaitCommand(250)
                        )
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
                                new DepositToStateCommand(arm, wrist, gripper, lift, "specimenToWall"),
                                new SequentialCommandGroup(
                                        new ActionCommand(redSpecEx_SubToLeftSpecZone, requirements),
                                        RedSpecEx_LeftSpecToLeftDepo
                                ),
                                new SequentialCommandGroup(
                                        new WaitCommand(1000),
                                        new InstantCommand(extendo::out),
                                        new InstantCommand(() -> intake.sIT.setPosition(BotPositions.INTAKE_WRIST_DOWN)),
                                        new WaitCommand(100),
                                        new InstantCommand(() -> intake.sIG.setPosition(BotPositions.INTAKE_ARM_DOWN))
                                ))

                );
                break;

            case "redSpecEx_MidSpecDepo":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> intake.sIG.setPosition(BotPositions.INTAKE_ARM_UP)),
                                        RedSpecEx_LeftDepoToMidSpec),
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new InstantCommand(() -> intake.sIG.setPosition(BotPositions.INTAKE_ARM_DOWN))),
                                        new SequentialCommandGroup(
                                                new WaitCommand(200),
                                                RedSpecEx_MidSpecToMidDepo))
                        )
                );
                break;

            case "redSpecEx_RightSpecDepo":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> intake.sIG.setPosition(BotPositions.INTAKE_ARM_UP)),
                                        RedSpecEx_MidDepoToRightSpec),
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new InstantCommand(() -> intake.sIG.setPosition(BotPositions.INTAKE_ARM_DOWN))),
                                        new SequentialCommandGroup(
                                                new WaitCommand(100),
                                                RedSpecEx_RightSpecToRightDepo))
                        )
                );
                break;


//RedBasket Commands

            case "redBasket_StartToBasketDepo":
                addCommands(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        RedBasket_StartToBasket,
                                        new InstantCommand(gripper::close),
                                        new SequentialCommandGroup(
                                                new WaitCommand(500),
                                                new DepositToStateCommand(arm, wrist, gripper, lift, "intakeToBasketHigh")
                                        )),
                                new WaitCommand(300),
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
                                        new InstantCommand(arm::specimen),
                                        new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH, BotPositions.LIFT_TOLERANCE),
                                        RedBasket_StartToSub,
                                        new SequentialCommandGroup(
                                                new WaitCommand(1800),
                                                new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH_CLIP, 70),
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
                                                RedBasket_BasketToRightSample),
                                        new SequentialCommandGroup(
                                                new WaitCommand(800), // was 1000
                                                new ParallelCommandGroup(
                                                        new InstantCommand(extendo::out),
                                                        new SequentialCommandGroup(
                                                                new InstantCommand(() -> intake.sIT.setPosition(BotPositions.INTAKE_WRIST_DOWN)),
                                                                new WaitCommand(100),
                                                                new InstantCommand(() -> intake.sIG.setPosition(BotPositions.INTAKE_ARM_DOWN)))
                                                ),

                                                new DepositToStateCommand(arm, wrist, gripper, lift, "basketToIntake")
                                        )),

                                new ParallelCommandGroup(
                                        new IntakeGetSampleCommand(intake),
                                        new SequentialCommandGroup(
                                                new WaitCommand(250),
                                                RedBasket_RightSampleIntake))

                                //   new IntakeGetSampleCommand(intake))//runs until sample is acquired
                        )
                );
                break;

            case "redBasket_ScoreRightSample":
                addCommands(
                        new SequentialCommandGroup(
                                new SequentialCommandGroup(
                                        new InstantCommand(intake::transferPosition),
                                        new InstantCommand(() -> intake.sIW.setPower(.17)),
                                        new InstantCommand(() -> intake.sIO.setPower(.17)),
                                        new WaitCommand(500),
                                        new InstantCommand(intake::stop),
                                        new InstantCommand(extendo::in)
                                ),
                                new ParallelCommandGroup(
//                                        new SequentialCommandGroup(
//                                                new WaitCommand(250),
//                                                new InstantCommand(intake::transfer)),
                                        new GripperAutoCloseCommand(gripper)),
                                new WaitCommand(500),
                                new InstantCommand(intake::stop),
                                new ParallelCommandGroup(
                                        new DepositToStateCommand(arm, wrist, gripper, lift, "intakeToBasketHigh"),
                                        new SequentialCommandGroup(
                                                new WaitCommand(800),//500
                                                RedBasket_RightSampleToBasket)),
                                new InstantCommand(gripper::open)
                        )
                );
                break;

            case "redBasket_IntakeMidSample":
                addCommands(
                        new SequentialCommandGroup(
                                //aware if the gripper will hit the basket
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new WaitCommand(200),
                                                RedBasket_BasketToMidSample),
                                        new SequentialCommandGroup(
                                                new WaitCommand(800),
                                                new ParallelCommandGroup(
                                                        new InstantCommand(extendo::out),
                                                        new SequentialCommandGroup(
                                                                new InstantCommand(() -> intake.sIT.setPosition(BotPositions.INTAKE_WRIST_DOWN)),
                                                                new WaitCommand(100),
                                                                new InstantCommand(() -> intake.sIG.setPosition(BotPositions.INTAKE_ARM_DOWN)))
                                                ),

                                                new DepositToStateCommand(arm, wrist, gripper, lift, "basketToIntake")
                                        )),

                                new ParallelCommandGroup(
                                        new IntakeGetSampleCommand(intake),
                                        new SequentialCommandGroup(
                                                new WaitCommand(250),
                                                RedBasket_MidSampleIntake))

                                //   new IntakeGetSampleCommand(intake))//runs until sample is acquired
                        )
                );
                break;

            case "redBasket_ScoreMidSample":
                addCommands(
                        new SequentialCommandGroup(
                                new SequentialCommandGroup(
                                        new InstantCommand(intake::transferPosition),
                                        new InstantCommand(() -> intake.sIW.setPower(.17)),
                                        new InstantCommand(() -> intake.sIO.setPower(.17)),
                                        new WaitCommand(500),
                                        new InstantCommand(intake::stop),
                                        new InstantCommand(extendo::in)
                                ),
                                new ParallelCommandGroup(
//                                        new SequentialCommandGroup(
//                                                new WaitCommand(250),
//                                                new InstantCommand(intake::transfer)),
                                        new GripperAutoCloseCommand(gripper)),
                                new WaitCommand(500),
                                new InstantCommand(intake::stop),
                                new ParallelCommandGroup(
                                        new DepositToStateCommand(arm, wrist, gripper, lift, "intakeToBasketHigh"),
                                        new SequentialCommandGroup(
                                                new WaitCommand(800),//400
                                                RedBasket_MidSampleToBasket)),
                                new InstantCommand(gripper::open)
                        )
                );
                break;

            case "redBasket_IntakeLeftSample":
                addCommands(
                        new SequentialCommandGroup(
                                //aware if the gripper will hit the basket
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new WaitCommand(200),
                                                RedBasket_BasketToLeftSample),
                                        new SequentialCommandGroup(
                                        new WaitCommand(800),
                                        new ParallelCommandGroup(
                                                new InstantCommand(extendo::out),
                                                new SequentialCommandGroup(
                                                        new InstantCommand(() -> intake.sIT.setPosition(BotPositions.INTAKE_WRIST_DOWN)),
                                                        new WaitCommand(100),
                                                        new InstantCommand(() -> intake.sIG.setPosition(BotPositions.INTAKE_ARM_DOWN)))
                                        ),
                                        new DepositToStateCommand(arm, wrist, gripper, lift, "basketToIntake")
                                )),

                                new ParallelCommandGroup(
                                        new IntakeGetSampleCommand(intake),
                                        new SequentialCommandGroup(
                                                new WaitCommand(250),
                                                RedBasket_LeftSampleIntake))

                                //   new IntakeGetSampleCommand(intake))//runs until sample is acquired
                        )
                );
                break;

            case "redBasket_ScoreLeftSample":
                addCommands(
                        new SequentialCommandGroup(
                                new SequentialCommandGroup(
                                        new InstantCommand(intake::transferPosition),
                                        new InstantCommand(() -> intake.sIW.setPower(.17)),
                                        new InstantCommand(() -> intake.sIO.setPower(.17)),
                                        new WaitCommand(500),
                                        new InstantCommand(intake::stop),
                                        new InstantCommand(extendo::in)
                                ),
                                new ParallelCommandGroup(
//                                        new SequentialCommandGroup(
//                                                new WaitCommand(250),
//                                                new InstantCommand(intake::transfer)),
                                        new GripperAutoCloseCommand(gripper)),
                                new WaitCommand(500),
                                new InstantCommand(intake::stop),
                                new ParallelCommandGroup(
                                        new DepositToStateCommand(arm, wrist, gripper, lift, "intakeToBasketHigh"),
                                        new SequentialCommandGroup(
                                                new WaitCommand(800),//500
                                                RedBasket_LeftSampleToBasket)),
                                new InstantCommand(gripper::open)
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

