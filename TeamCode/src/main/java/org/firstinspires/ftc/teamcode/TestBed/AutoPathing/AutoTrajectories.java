package org.firstinspires.ftc.teamcode.TestBed.AutoPathing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Subsystem;


import org.firstinspires.ftc.teamcode.TestBed.Tuning.MecanumDrive;

import java.util.Set;


public class AutoTrajectories {

    //Red

    //Red Basket Poses
    public static final Pose2d redBasket_StartPos = new Pose2d(-15, -64, Math.toRadians(270));
    public static final Pose2d redBasket_BasketDrop = new Pose2d(-56,-53, Math.toRadians(45));
    public static final Pose2d redBasket_SubDrop = new Pose2d(-10, -36, Math.toRadians(270));
    public static final Pose2d redBasket_RightSampleZonePos =new Pose2d(-45, -40, Math.toRadians(90));
    public static final Pose2d redBasket_MidSampleZonePos = new Pose2d(-55,-40, Math.toRadians(90));
    public static final Pose2d redBasket_LeftSampleZonePos = new Pose2d(-53,-40, Math.toRadians(130));
    public static final Pose2d redBasket_AscentPos = new Pose2d(-28, -11, Math.toRadians(180));

    //Red Specimen Poses
    public static final Pose2d redSpec_StartPos = new Pose2d(15, -64, Math.toRadians(270));
    public static final Pose2d redSpec_SubDepoPos = new Pose2d(10, -36, Math.toRadians(270));
    public static final Pose2d redSpec_ObsSpecPos = new Pose2d(33, -63, Math.toRadians(270));
    public static final Pose2d redSpec_RightSampleZonePos = new Pose2d(58, -45, Math.toRadians(75));
//    public static final Pose2d redSpec_MidSampleZonePos = new Pose2d(58, -45, Math.toRadians(90));
    public static final Pose2d redSpec_LeftMidSampleZonePos = new Pose2d(50, -45, Math.toRadians(90));
    public static final Pose2d redSpec_ObsSlideParkPos = new Pose2d(40,-18,Math.toRadians(270)); // might not need this
    public static final Pose2d redSpec_ObsParkPos = new Pose2d(36,15,Math.toRadians(270));


    //blue

    //Blue Basket Pose
    public static final Pose2d blueBasket_StartPos = new Pose2d(-15, 64, Math.toRadians(270));
    public static final Pose2d blueBasket_BasketDrop = new Pose2d(-56,53, Math.toRadians(45));
    public static final Pose2d blueBasket_SubDrop = new Pose2d(-10, 36, Math.toRadians(270));
    public static final Pose2d blueBasket_RightSampleZonePos =new Pose2d(-45, 40, Math.toRadians(90));
    public static final Pose2d blueBasket_MidSampleZonePos = new Pose2d(-55,40, Math.toRadians(90));
    public static final Pose2d blueBasket_LeftSampleZonePos = new Pose2d(-53,40, Math.toRadians(130));
    public static final Pose2d blueBasket_AscentPos = new Pose2d(-28, 11, Math.toRadians(180));

    //Blue Specimen Pose
    public static final Pose2d blueSpec_StartPos = new Pose2d(15, 64, Math.toRadians(270));
    public static final Pose2d blueSpec_SubDepoPos = new Pose2d(10, 36, Math.toRadians(270));
    public static final Pose2d blueSpec_ObsSpecPos = new Pose2d(33, 63, Math.toRadians(270));
    public static final Pose2d blueSpec_RightSampleZonePos = new Pose2d(58, 45, Math.toRadians(75));
//    public static final Pose2d blueSpec_MidSampleZonePos = new Pose2d(58, 45, Math.toRadians(90));
    public static final Pose2d blueSpec_LeftMidSampleZonePos = new Pose2d(50, 45, Math.toRadians(90));
    public static final Pose2d blueSpec_ObsParkPos = new Pose2d(36,15,Math.toRadians(270));




//Actions

   //Red

    //Red Spec Actions
    public static Action redSpec_StartToSub;
    public static Action redSpec_SubToLeftSpec;
    public static Action redSpec_LeftSpecToObs;
    public static Action redSpec_ObsToSub;
    public static Action redSpec_SubToObs;
    public static Action redSpec_ObsToRightSpec;
    public static Action redSpec_RightSpecToObs;
    public static Action redSpec_Park;

    //Red Basket Actions
    public static Action redBasket_StartToSub;
    public static Action redBasket_SubToRightSample;
    public static Action redBasket_RightSampleToBasket;
    public static Action redBasket_ToMidSample;
    public static Action redBasket_MidSampleToBasket;
    public static Action redBasket_BasketToLeftSample;
    public static Action redBasket_LeftSampleToBasket;
    public static Action redBasket_BasketToAscent;

//    //blue
//
//    //Blue Spec Action
//    public static Action BlueSpec_StartToSub;
//    public static Action BlueSpec_SubToLeftSpec;
//    public static Action BlueSpec_LeftSpecToObs;
//    public static Action BlueSpec_ObsToSub;
//    public static Action BlueSpec_SubToObs;
//    public static Action BlueSpec_ObsToRightSpec;
//    public static Action BlueSpec_RightSpecToObs;
//    public static Action BlueSpec_Park;
//
//    //Blue Basket Actions
//    public static Action BlueBasket_StartToSub;
//    public static Action BlueBasket_SubToRightSample;
//    public static Action BlueBasket_RightSampleToBasket;
//    public static Action BlueBasket_BasketToMidSample;
//    public static Action BlueBasket_MidSampleToBasket;
//    public static Action BlueBasket_BasketToLeftSample;
//    public static Action BlueBasket_LeftSampleToBasket;
//    public static Action BlueBasket_BasketToAscent;



    public static void generateTrajectories(MecanumDrive drive) {

        //red soecimen auto

        redSpec_StartToSub =
                drive.actionBuilder(redSpec_StartPos)
                        .setTangent(90)
                        .splineToLinearHeading(redSpec_SubDepoPos, Math.toRadians(90))
                        .build();


        redSpec_SubToLeftSpec =
                drive.actionBuilder(redSpec_SubDepoPos)
                        .setTangent(200)
                        .splineToLinearHeading(redSpec_LeftMidSampleZonePos, Math.toRadians(0))
                        .build();

        redSpec_LeftSpecToObs =
                drive.actionBuilder(redSpec_LeftMidSampleZonePos)
                        .setTangent(0)
                        .splineToLinearHeading(redSpec_ObsSpecPos, Math.toRadians(0))
                        .build();

        redSpec_ObsToSub =
                drive.actionBuilder(redSpec_ObsSpecPos)
                        .setTangent(90)
                        .splineToLinearHeading(redSpec_SubDepoPos, Math.toRadians(90))
                        .build();

        redSpec_SubToObs =
                drive.actionBuilder(redSpec_SubDepoPos)
                        .setTangent(135)
                        .splineToLinearHeading(redSpec_ObsSpecPos, Math.toRadians(180))
                        .build();

        redSpec_ObsToRightSpec =
                drive.actionBuilder(redSpec_ObsSpecPos)
                        .setTangent(270)
                        .splineToLinearHeading(redSpec_RightSampleZonePos, Math.toRadians(0))
                        .build();

        redSpec_RightSpecToObs =
                drive.actionBuilder(redSpec_RightSampleZonePos)
                        .setTangent(180)
                        .splineToLinearHeading(redSpec_ObsSpecPos, Math.toRadians(270))
                        .build();

        redSpec_Park =
                drive.actionBuilder(redSpec_SubDepoPos)
                        .lineToX(40)
                        .setTangent(300)
                        .splineToLinearHeading(redSpec_ObsParkPos, Math.toRadians(270))
                        .build();

//lower case


        //red basket auto

        redBasket_StartToSub =
                drive.actionBuilder(redBasket_StartPos)
                        .setTangent(70)
                        .splineToLinearHeading(redBasket_SubDrop, Math.toRadians(90))
                        .build();

        redBasket_SubToRightSample =
                drive.actionBuilder(redBasket_SubDrop)
                        .setTangent(180)
                        .splineToLinearHeading(redBasket_RightSampleZonePos, Math.toRadians(180))
                        .build();

        redBasket_RightSampleToBasket =
                drive.actionBuilder(redBasket_RightSampleZonePos)
                        .setTangent(180)
                        .splineToLinearHeading(redBasket_BasketDrop, Math.toRadians(270))
                        .build();

        redBasket_ToMidSample =
                drive.actionBuilder(redBasket_BasketDrop)
                        .setTangent(45)
                        .splineToLinearHeading(redBasket_MidSampleZonePos, Math.toRadians(90))
                        .build();

        redBasket_MidSampleToBasket =
                drive.actionBuilder(redBasket_MidSampleZonePos)
                        .setTangent(180)
                        .splineToLinearHeading(redBasket_BasketDrop, Math.toRadians(270))
                        .build();

        redBasket_BasketToLeftSample =
                drive.actionBuilder(redBasket_BasketDrop)
                        .setTangent(45)
                        .splineToLinearHeading(redBasket_LeftSampleZonePos, Math.toRadians(90))
                        .build();

        redBasket_LeftSampleToBasket =
                drive.actionBuilder(redBasket_LeftSampleZonePos)
                        .setTangent(180)
                        .splineToLinearHeading(redBasket_BasketDrop, Math.toRadians(270))
                        .build();

        redBasket_BasketToAscent =
                drive.actionBuilder(redBasket_BasketDrop)
                        .setTangent(0)
                        .splineToLinearHeading(redBasket_AscentPos, Math.toRadians(0))
                        .build();

    }}
//        //blue specimen Auto
//        BlueSpec_StartToSub =
//                drive.actionBuilder(blueSpec_StartPos)
//                        .setTangent(90)
//                        .splineToLinearHeading(blueSpec_SubDepoPos, Math.toRadians(90))
//                        .build();
//
//
//        BlueSpec_SubToLeftSpec =
//                drive.actionBuilder(blueSpec_SubDepoPos)
//                        .setTangent(200)
//                        .splineToLinearHeading(blueSpec_LeftMidSampleZonePos, Math.toRadians(0))
//                        .build();
//
//        BlueSpec_LeftSpecToObs =
//                drive.actionBuilder(blueSpec_LeftMidSampleZonePos)
//                        .setTangent(0)
//                        .splineToLinearHeading(blueSpec_ObsSpecPos, Math.toRadians(0))
//                        .build();
//
//        BlueSpec_ObsToSub =
//                drive.actionBuilder(blueSpec_ObsSpecPos)
//                        .setTangent(90)
//                        .splineToLinearHeading(blueSpec_SubDepoPos, Math.toRadians(90))
//                        .build();
//
//        RedSpec_SubToObs =
//                drive.actionBuilder(blueSpec_SubDepoPos)
//                        .setTangent(135)
//                        .splineToLinearHeading(blueSpec_ObsSpecPos, Math.toRadians(180))
//                        .build();
//
//        BlueSpec_ObsToRightSpec =
//                drive.actionBuilder(blueSpec_ObsSpecPos)
//                        .setTangent(270)
//                        .splineToLinearHeading(blueSpec_RightSampleZonePos, Math.toRadians(0))
//                        .build();
//
//        BlueSpec_RightSpecToObs =
//                drive.actionBuilder(blueSpec_RightSampleZonePos)
//                        .setTangent(180)
//                        .splineToLinearHeading(blueSpec_ObsSpecPos, Math.toRadians(270))
//                        .build();
//
//        BlueSpec_Park =
//                drive.actionBuilder(blueSpec_SubDepoPos)
//                        .lineToX(40)
//                        .setTangent(300)
//                        .splineToLinearHeading(blueSpec_ObsParkPos, Math.toRadians(270))
//                        .build();
//
//
//
//        //blue basket auto
//
//        BlueBasket_StartToSub =
//                drive.actionBuilder(redBasket_StartPos)
//                        .setTangent(70)
//                        .splineToLinearHeading(redBasket_SubDrop, Math.toRadians(90))
//                        .build();
//
//        BlueBasket_SubToRightSample =
//                drive.actionBuilder(redBasket_SubDrop)
//                        .setTangent(180)
//                        .splineToLinearHeading(redBasket_RightSampleZonePos, Math.toRadians(180))
//                        .build();
//
//        BlueBasket_RightSampleToBasket =
//                drive.actionBuilder(redBasket_RightSampleZonePos)
//                        .setTangent(180)
//                        .splineToLinearHeading(redBasket_BasketDrop, Math.toRadians(270))
//                        .build();
//
//        BlueBasket_BasketToMidSample =
//                drive.actionBuilder(redBasket_BasketDrop)
//                        .setTangent(45)
//                        .splineToLinearHeading(redBasket_MidSampleZonePos, Math.toRadians(90))
//                        .build();
//
//        BlueBasket_MidSampleToBasket =
//                drive.actionBuilder(redBasket_MidSampleZonePos)
//                        .setTangent(180)
//                        .splineToLinearHeading(redBasket_BasketDrop, Math.toRadians(270))
//                        .build();
//
//        BlueBasket_BasketToLeftSample =
//                drive.actionBuilder(redBasket_BasketDrop)
//                        .setTangent(45)
//                        .splineToLinearHeading(redBasket_LeftSampleZonePos, Math.toRadians(90))
//                        .build();
//
//        BlueBasket_LeftSampleToBasket =
//                drive.actionBuilder(redBasket_LeftSampleZonePos)
//                        .setTangent(180)
//                        .splineToLinearHeading(redBasket_BasketDrop, Math.toRadians(270))
//                        .build();
//
//        BlueBasket_BasketToAscent =
//                drive.actionBuilder(redBasket_BasketDrop)
//                        .setTangent(0)
//                        .splineToLinearHeading(redBasket_AscentPos, Math.toRadians(0))
//                        .build();
//
//
//
