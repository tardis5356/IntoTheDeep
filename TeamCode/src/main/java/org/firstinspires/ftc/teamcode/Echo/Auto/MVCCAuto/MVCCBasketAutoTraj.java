package org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto;


import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;

import org.firstinspires.ftc.teamcode.Echo.Auto.Tuning.MecanumDriveBasket;

import java.util.Arrays;


public class MVCCBasketAutoTraj {

    public static final Pose2d redBasket_SampleStartPos = new Pose2d(-39.25, -63, Math.toRadians(0));
    public static final Pose2d redBasket_SpecimenStartPos = new Pose2d(-16, -64, Math.toRadians(90));
    public static final Pose2d redBasket_SubDepoPos = new Pose2d(-2, -32, Math.toRadians(90));
    public static final Pose2d redBasket_BasketDrop = new Pose2d(-60,-59, Math.toRadians(45));

    public static final Pose2d redBasket_BasketDrop1 = new Pose2d(-59.5,-60.5, Math.toRadians(45));
    public static final Pose2d redBasket_RightSampleIntakePos =new Pose2d(-47, -55.5, Math.toRadians(90));
//    public static final Pose2d redBasket_RightSampleZonePos =new Pose2d(-46, -50, Math.toRadians(93));
    public static final Pose2d redBasket_MidSampleIntakePos = new Pose2d(-58,-53.5, Math.toRadians(90));
//    public static final Pose2d redBasket_MidSampleZonePos = new Pose2d(-54.5,-49, Math.toRadians(100));
    public static final Pose2d redBasket_LeftSampleIntakePos = new Pose2d(-56,-54, Math.toRadians(110));
//    public static final Pose2d redBasket_LeftSampleZonePos = new Pose2d(-52,-52, Math.toRadians(120));
    public static final Pose2d redBasket_AscentParkPos = new Pose2d(-21, -12, Math.toRadians(180));

    public static final Pose2d redBasket_SubPos1B = new Pose2d(-35, -16, Math.toRadians(0));

    public static final Pose2d redBasket_SubPos2B = new Pose2d(-35, -10, Math.toRadians(0));
    public static final Pose2d redBasket_SubPos3B = new Pose2d(-35, -4, Math.toRadians(0));

    public static final Pose2d redBasket_SubPos1A = new Pose2d(-40, -16, Math.toRadians(0));

    public static final Pose2d redBasket_SubPos2A = new Pose2d(-40, -10, Math.toRadians(0));
    public static final Pose2d redBasket_SubPos3A = new Pose2d(-40, -4, Math.toRadians(0));

    public static final Pose2d redBasket_WaypointPos = new Pose2d(-40,0, Math.toRadians(0));





    //Actions

    //    public static Action redBasket_SubToRightSample;
    public static Action redBasket_StartToSub;
    public static Action redBasket_SubToRightSample;
    public static Action redBasket_StartToBasket;
    public static Action redBasket_BasketToRightSample;
    public static Action redBasket_RightSampleIntake;
    public static Action redBasket_RightSampleToBasket;
    public static Action redBasket_BasketToMidSample;
    public static Action redBasket_MidSampleIntake;
    public static Action redBasket_MidSampleToBasket;
    public static Action redBasket_BasketToLeftSample;
    public static Action redBasket_LeftSampleIntake;
    public static Action redBasket_LeftSampleToBasket;
    public static Action redBasket_BasketToAscentPark;
    public static Action redBasket_BasketToSub1A;
    public static Action redBasket_BasketToSub2A;
    public static Action redBasket_BasketToSub3A;
    public static Action redBasket_BasketToSub1B;
    public static Action redBasket_BasketToSub2B;
    public static Action redBasket_BasketToSub3B;

    public static Action redBasket_BasketToSub1A_2;
    public static Action redBasket_BasketToSub2A_2;
    public static Action redBasket_BasketToSub3A_2;
    public static Action redBasket_BasketToSub1B_2;
    public static Action redBasket_BasketToSub2B_2;
    public static Action redBasket_BasketToSub3B_2;
    public static Action redBasket_SubToBasket;

    public static Action redBasket_SubToBasket_2;
    public static Action redBasket_SubToSubIntakeA;
    public static Action redBasket_SubToSubIntakeB;

    public static Action redBasket_SubToWaypoint;
    public static MinVelConstraint IntakeConstraint = new MinVelConstraint(
            Arrays.asList(
                    new TranslationalVelConstraint(80),//inches per second
                    new AngularVelConstraint(3.5) // remember the units you're working in, especially for angular constraints!
            )
    );

    public static AccelConstraint IntakeAccelConstraint = new ProfileAccelConstraint(-40, 60);

    public static MinVelConstraint RightIntakeConstraint = new MinVelConstraint(
            Arrays.asList(
                    new TranslationalVelConstraint(60),//inches per second
                    new AngularVelConstraint(4) // remember the units you're working in, especially for angular constraints!
            )
    );


    public static void generateTrajectories(MecanumDriveBasket drive) {

        redBasket_StartToSub =
                drive.actionBuilder(redBasket_SpecimenStartPos)
                        .setTangent(Math.toRadians(120)) //90
                        .splineToLinearHeading(redBasket_SubDepoPos, Math.toRadians(90))
                        .build();

        redBasket_SubToRightSample =
                drive.actionBuilder(redBasket_SubDepoPos)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(redBasket_RightSampleIntakePos, Math.toRadians(180))
                        .build();

        redBasket_StartToBasket =
                drive.actionBuilder(redBasket_SampleStartPos)
                        .setTangent(90)
                        .splineToLinearHeading(redBasket_BasketDrop, Math.toRadians(225))
                        .build();

        redBasket_BasketToRightSample =
                drive.actionBuilder(redBasket_BasketDrop)
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(redBasket_RightSampleIntakePos,Math.toRadians(90), RightIntakeConstraint, IntakeAccelConstraint)
                        .build();

//        redBasket_RightSampleIntake =
//                drive.actionBuilder(redBasket_RightSampleIntakePos)
//                        .setTangent(90)
//                        .splineToLinearHeading(redBasket_RightSampleZonePos,Math.toRadians(90))
//                        .build();

        redBasket_RightSampleToBasket =
                drive.actionBuilder(redBasket_RightSampleIntakePos)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(redBasket_BasketDrop, Math.toRadians(225))
                        .build();

        redBasket_BasketToMidSample =
                drive.actionBuilder(redBasket_BasketDrop)
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(redBasket_MidSampleIntakePos, Math.toRadians(90), IntakeConstraint, IntakeAccelConstraint)
                        .build();

//        redBasket_MidSampleIntake =
//                drive.actionBuilder(redBasket_MidSampleIntakePos)
//                        .setTangent(90)
//                        .splineToLinearHeading(redBasket_MidSampleZonePos,Math.toRadians(90))
//                        .build();

        redBasket_MidSampleToBasket =
                drive.actionBuilder(redBasket_MidSampleIntakePos)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(redBasket_BasketDrop, Math.toRadians(225))
                        .build();

        redBasket_BasketToLeftSample =
                drive.actionBuilder(redBasket_BasketDrop)
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(redBasket_LeftSampleIntakePos, Math.toRadians(270), IntakeConstraint, IntakeAccelConstraint)
                        .build();

//        redBasket_LeftSampleIntake =
//                drive.actionBuilder(redBasket_LeftSampleIntakePos)
//                        .setTangent(90)
//                        .splineToLinearHeading(redBasket_LeftSampleZonePos,Math.toRadians(90))
//                        .build();
        redBasket_LeftSampleToBasket =
                drive.actionBuilder(redBasket_LeftSampleIntakePos)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(redBasket_BasketDrop, Math.toRadians(270))
                        .build();

        redBasket_BasketToAscentPark =
                drive.actionBuilder(redBasket_BasketDrop)
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(redBasket_AscentParkPos, Math.toRadians(315))
                        .build();

        redBasket_BasketToSub1A =
                drive.actionBuilder(redBasket_BasketDrop)
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(redBasket_SubPos1A, Math.toRadians(315))
                        .build();

        redBasket_BasketToSub1B =
                drive.actionBuilder(redBasket_BasketDrop)
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(redBasket_SubPos1B, Math.toRadians(315))
                        .build();
        redBasket_BasketToSub2A =
                drive.actionBuilder(redBasket_BasketDrop)
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(redBasket_SubPos2A, Math.toRadians(315))
                        .build();
        redBasket_BasketToSub2B =
                drive.actionBuilder(redBasket_BasketDrop)
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(redBasket_SubPos2B, Math.toRadians(315))
                        .build();
        redBasket_BasketToSub3A =
                drive.actionBuilder(redBasket_BasketDrop)
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(redBasket_SubPos3A, Math.toRadians(315))
                        .build();
        redBasket_BasketToSub3B =
                drive.actionBuilder(redBasket_BasketDrop)
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(redBasket_SubPos3B, Math.toRadians(315))
                        .build();

        redBasket_BasketToSub1A_2 =
                drive.actionBuilder(redBasket_BasketDrop1)
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(redBasket_SubPos1A, Math.toRadians(315))
                        .build();

        redBasket_BasketToSub1B_2 =
                drive.actionBuilder(redBasket_BasketDrop1)
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(redBasket_SubPos1B, Math.toRadians(315))
                        .build();
        redBasket_BasketToSub2A_2 =
                drive.actionBuilder(redBasket_BasketDrop1)
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(redBasket_SubPos2A, Math.toRadians(315))
                        .build();
        redBasket_BasketToSub2B_2 =
                drive.actionBuilder(redBasket_BasketDrop1)
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(redBasket_SubPos2B, Math.toRadians(315))
                        .build();
        redBasket_BasketToSub3A_2 =
                drive.actionBuilder(redBasket_BasketDrop1)
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(redBasket_SubPos3A, Math.toRadians(315))
                        .build();
        redBasket_BasketToSub3B_2 =
                drive.actionBuilder(redBasket_BasketDrop1)
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(redBasket_SubPos3B, Math.toRadians(315))
                        .build();

        redBasket_SubToSubIntakeA = drive.actionBuilder(drive.pose)
                .lineToX(-40)
                .build();

        redBasket_SubToSubIntakeB = drive.actionBuilder(drive.pose)
                .lineToX(-35)
                .build();

        redBasket_SubToWaypoint = drive.actionBuilder(drive.pose)

                .lineToX(-50)
                .build();

        redBasket_SubToBasket = drive.actionBuilder(new Pose2d(-50,drive.pose.position.y, Math.toRadians(0)))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(redBasket_BasketDrop1, Math.toRadians(225))
                .build();

        redBasket_SubToBasket_2 = drive.actionBuilder(new Pose2d(drive.pose.position.x,drive.pose.position.y, Math.toRadians(0)))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(redBasket_BasketDrop1, Math.toRadians(225))
                .build();


    }}