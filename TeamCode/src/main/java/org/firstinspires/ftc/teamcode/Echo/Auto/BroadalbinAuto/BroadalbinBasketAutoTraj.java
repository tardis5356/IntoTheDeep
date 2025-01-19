package org.firstinspires.ftc.teamcode.Echo.Auto.BroadalbinAuto;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.Echo.Auto.Tuning.MecanumDriveBasket;


public class BroadalbinBasketAutoTraj {

    public static final Pose2d redBasket_SampleStartPos = new Pose2d(-40, -62.5, Math.toRadians(0));
    public static final Pose2d redBasket_SpecimenStartPos = new Pose2d(-16, -64, Math.toRadians(90));
    public static final Pose2d redBasket_SubDepoPos = new Pose2d(-2, -32, Math.toRadians(90));
    public static final Pose2d redBasket_BasketDrop = new Pose2d(-57,-58, Math.toRadians(45));
    public static final Pose2d redBasket_RightSampleIntakePos =new Pose2d(-48, -62, Math.toRadians(93));
    public static final Pose2d redBasket_RightSampleZonePos =new Pose2d(-48, -56, Math.toRadians(93));
    public static final Pose2d redBasket_MidSampleIntakePos = new Pose2d(-51.5,-62, Math.toRadians(105));
    public static final Pose2d redBasket_MidSampleZonePos = new Pose2d(-51.5,-56, Math.toRadians(105));
    public static final Pose2d redBasket_LeftSampleIntakePos = new Pose2d(-55,-61, Math.toRadians(120));
    public static final Pose2d redBasket_LeftSampleZonePos = new Pose2d(-55,-54, Math.toRadians(120));
    public static final Pose2d redBasket_AscentParkPos = new Pose2d(-26, -12, Math.toRadians(180));



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



    public static void generateTrajectories(MecanumDriveBasket drive) {

        redBasket_StartToSub =
                drive.actionBuilder(redBasket_SpecimenStartPos)
                        .setTangent(Math.toRadians(90))
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
                        .splineToLinearHeading(redBasket_BasketDrop, Math.toRadians(180))
                        .build();

        redBasket_BasketToRightSample =
                drive.actionBuilder(redBasket_BasketDrop)
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(redBasket_RightSampleIntakePos,Math.toRadians(270))
                        .build();

        redBasket_RightSampleIntake =
                drive.actionBuilder(redBasket_RightSampleIntakePos)
                        .setTangent(90)
                        .splineToLinearHeading(redBasket_RightSampleZonePos,Math.toRadians(90))
                        .build();

        redBasket_RightSampleToBasket =
                drive.actionBuilder(redBasket_RightSampleZonePos)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(redBasket_BasketDrop, Math.toRadians(220))
                        .build();

        redBasket_BasketToMidSample =
                drive.actionBuilder(redBasket_BasketDrop)
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(redBasket_MidSampleIntakePos, Math.toRadians(270))
                        .build();

        redBasket_MidSampleIntake =
                drive.actionBuilder(redBasket_MidSampleIntakePos)
                        .setTangent(90)
                        .splineToLinearHeading(redBasket_MidSampleZonePos,Math.toRadians(90))
                        .build();

        redBasket_MidSampleToBasket =
                drive.actionBuilder(redBasket_MidSampleZonePos)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(redBasket_BasketDrop, Math.toRadians(270))
                        .build();

        redBasket_BasketToLeftSample =
                drive.actionBuilder(redBasket_BasketDrop)
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(redBasket_LeftSampleIntakePos, Math.toRadians(270))
                        .build();

        redBasket_LeftSampleIntake =
                drive.actionBuilder(redBasket_LeftSampleIntakePos)
                        .setTangent(90)
                        .splineToLinearHeading(redBasket_LeftSampleZonePos,Math.toRadians(90))
                        .build();
        redBasket_LeftSampleToBasket =
                drive.actionBuilder(redBasket_LeftSampleZonePos)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(redBasket_BasketDrop, Math.toRadians(300))
                        .build();

        redBasket_BasketToAscentPark =
                drive.actionBuilder(redBasket_BasketDrop)
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(redBasket_AscentParkPos, Math.toRadians(0))
                        .build();

    }}