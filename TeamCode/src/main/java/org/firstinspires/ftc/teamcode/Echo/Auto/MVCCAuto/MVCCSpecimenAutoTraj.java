package org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.DemoBots.primus.Pose;
import org.firstinspires.ftc.teamcode.Echo.Auto.Tuning.MecanumDriveSpecimen;


public class MVCCSpecimenAutoTraj {

    public static final Pose2d redSpec_StartPos = new Pose2d(8, -64, Math.toRadians(90));
    public static final Pose2d redSpec_SubDepoPos = new Pose2d(-4, -34.6, Math.toRadians(90)); //two inches from the sub
    public static final Pose2d redSpec_ObsSpecPos = new Pose2d(35.5, -62, Math.toRadians(90));
    public static final Pose2d redSpec_ObsSpecPos1 = new Pose2d(35.5, -62, Math.toRadians(90));
    public static final Pose2d redSpec_ObsSpecPos2 = new Pose2d(35.5, -62, Math.toRadians(90));
    public static final Pose2d redSpec_ObsSpecPos3 = new Pose2d(37, -62, Math.toRadians(90));
    public static final Pose2d redSpec_MidPointPos = new Pose2d(36, -40, Math.toRadians(90));
    public static final Pose2d redSpec_SubDepoPos1 = new Pose2d(-2, -34.6, Math.toRadians(90)); //y=-29
    public static final Pose2d redSpec_SubDepoPos2 = new Pose2d(0, -34.6, Math.toRadians(90)); //y=-29
    public static final Pose2d redSpec_SubDepoPos3 = new Pose2d(2, -34.6, Math.toRadians(90)); //y=-29
    public static final Pose2d redSpec_SubDepoPos4 = new Pose2d(4,-34.6, Math.toRadians(90)); //y=-29
    public static final Pose2d redSpecEx_LeftSpecZonePos = new Pose2d(25, -44, Math.toRadians(55));
    public static final Pose2d redSpecEx_LeftSpecDepoPos = new Pose2d(29.7, -50, Math.toRadians(310));
    public static final Pose2d redSpecEx_MidSpecZonePos = new Pose2d(32, -37, Math.toRadians(45));
    public static final Pose2d redSpecEx_MidSpecDepoPos = new Pose2d(35, -47, Math.toRadians(310));//320
    public static final Pose2d redSpecEx_RightSpecZonePos = new Pose2d(42.6, -39, Math.toRadians(55));
    public static final Pose2d redSpecEx_RightSpecDepoPos = new Pose2d(43.5, -58, Math.toRadians(303)); //37,-58,335
    public static final Pose2d redSpecEx_ObsPrepPos = new Pose2d(35, -58, Math.toRadians(90));

    public static final Pose2d  redBasket_BasketDrop = new Pose2d(-57,-58, Math.toRadians(45));


    //Actions
    public static Action redSpec_StartToSub;
    public static Action redSpec_SubToMidPoint;
    public static Action redSpec_ObsToSub1;
    public static Action redSpec_ObsToSub2;
    public static Action redSpec_ObsToSub3;
    public static Action redSpec_ObsToSub4;

    public static Action redSpec_SubToObs;
    public static Action redSpec_SubToObs2;
    public static Action redSpec_SubToObs3;
    public static Action redSpec_SubToObs4;
    public static Action redSpecEx_SubToLeftSpecZone;
    public static Action redSpecEx_LeftSpecToLeftDepo;
    public static Action redSpecEx_LeftDepoToMidSpec;
    public static Action redSpecEx_MidSpecToMidDepo;
    public static Action redSpecEx_MidDepoToRightSpec;
    public static Action redSpecEx_RightSpecToRightDepo;
    public static Action redSpecEx_RightDepoToObsPrep;
    public static Action redSpecEx_ObsPrepToObsSpec;

    public static Action teleop_CurrentToBasket;


    public static void generateTrajectories(MecanumDriveSpecimen drive) {

        redSpec_StartToSub =
                drive.actionBuilder(redSpec_StartPos)

                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(redSpec_SubDepoPos, Math.toRadians(90))
                        .build();


        redSpec_SubToMidPoint =
                drive.actionBuilder(redSpec_SubDepoPos)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(redSpec_MidPointPos, Math.toRadians(90))
                        .build();


        redSpec_SubToObs =
                drive.actionBuilder(redSpec_SubDepoPos)
                        .setTangent(Math.toRadians(315))
                        .splineToLinearHeading(redSpec_ObsSpecPos, Math.toRadians(270))
                        .build();

        redSpec_SubToObs2 =
                drive.actionBuilder(redSpec_SubDepoPos2)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(redSpec_ObsSpecPos1, Math.toRadians(270))
                        .build();

        redSpec_SubToObs3 =
                drive.actionBuilder(redSpec_SubDepoPos3)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(redSpec_ObsSpecPos2, Math.toRadians(270))
                        .build();

        redSpec_SubToObs4 =
                drive.actionBuilder(redSpec_SubDepoPos4)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(redSpec_ObsSpecPos3, Math.toRadians(270))
                        .build();


        redSpec_ObsToSub1 =
                drive.actionBuilder(redSpec_ObsSpecPos)
//                        .strafeTo(new Vector2d(-2.5, -29))
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(redSpec_SubDepoPos1, Math.toRadians(90))
                        .build();

        redSpec_ObsToSub2 =
                drive.actionBuilder(redSpec_ObsSpecPos1)
//                        .strafeTo(new Vector2d(0, -29))
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(redSpec_SubDepoPos2, Math.toRadians(90))
                        .build();

        redSpec_ObsToSub3 =
                drive.actionBuilder(redSpec_ObsSpecPos2)
//                        .strafeTo(new Vector2d(2, -29))
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(redSpec_SubDepoPos3, Math.toRadians(90))
                        .build();

        redSpec_ObsToSub4 =
                drive.actionBuilder(redSpec_ObsSpecPos3)
//                        .strafeTo(new Vector2d(4, -29))
                        .setTangent(90)
                        .splineToLinearHeading(redSpec_SubDepoPos4, Math.toRadians(90))
                        .build();

        redSpecEx_SubToLeftSpecZone =
                drive.actionBuilder(redSpec_SubDepoPos)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(redSpecEx_LeftSpecZonePos, Math.toRadians(0))
                        .build();

        redSpecEx_LeftSpecToLeftDepo =
                drive.actionBuilder(redSpecEx_LeftSpecZonePos)
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(redSpecEx_LeftSpecDepoPos, Math.toRadians(0))
                        .build();

        redSpecEx_LeftDepoToMidSpec =
                drive.actionBuilder(redSpecEx_LeftSpecDepoPos)
                        .setTangent(Math.toRadians(90))//0
                        .splineToLinearHeading(redSpecEx_MidSpecZonePos, Math.toRadians(0))
                        .build();


        redSpecEx_MidSpecToMidDepo =
                drive.actionBuilder(redSpecEx_MidSpecZonePos)
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(redSpecEx_MidSpecDepoPos, Math.toRadians(270))
                        .build();


        redSpecEx_MidDepoToRightSpec =
                drive.actionBuilder(redSpecEx_MidSpecDepoPos)
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(redSpecEx_RightSpecZonePos, Math.toRadians(0))
                        .build();

        redSpecEx_RightSpecToRightDepo =
                drive.actionBuilder(redSpecEx_RightSpecZonePos)
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(redSpecEx_RightSpecDepoPos, Math.toRadians(270))
                        .build();


        redSpecEx_RightDepoToObsPrep =
                drive.actionBuilder(redSpecEx_RightSpecDepoPos)
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(redSpecEx_ObsPrepPos, Math.toRadians(180))
                        .build();

        redSpecEx_ObsPrepToObsSpec =
                drive.actionBuilder(redSpecEx_ObsPrepPos)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(redSpec_ObsSpecPos, Math.toRadians(270))
                        .build();


    }
}