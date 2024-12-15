package org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.Echo.Auto.Tuning.MecanumDrive;


public class UticaAutoTrajectories {

    //Red


    //Red Specimen Poses
    public static final Pose2d redSpec_StartPos = new Pose2d(16, -64, Math.toRadians(90));
    //   public static final Pose2d redSpec_StartPos = new Pose2d(40, -64, Math.toRadians(90));
    public static final Pose2d redSpec_SubDepoPos = new Pose2d(0, -32, Math.toRadians(90));
    public static final Pose2d redSpec_ObsSpecPos = new Pose2d(42, -61, Math.toRadians(90));
//    public static final Pose2d redSpec_ObsSpecCheckPos = new Pose2d(48,-61,Math.toRadians(90));
    public static final Pose2d redSpec_LeftSpecZonePos = new Pose2d(46, -10, Math.toRadians(90));
    public static final Pose2d redSpec_LeftSpecDepoPos = new Pose2d(46, -56, Math.toRadians(90));
    public static final Pose2d redSpec_MidSpecZonePos = new Pose2d(53, -10, Math.toRadians(90));
    public static final Pose2d redSpec_MidSpecDepoPos = new Pose2d(53, -56, Math.toRadians(90));
    public static final Pose2d redSpec_RightSpecZonePos = new Pose2d(66, -14, Math.toRadians(90));
    public static final Pose2d redSpec_RightSpecDepoPos = new Pose2d(66, -66, Math.toRadians(90));
    public static final Pose2d redSpec_MidWayPos = new Pose2d(36,-40,Math.toRadians(90));

    //Red Basket Poses
    public static final Pose2d redBasket_StartPos = new Pose2d(-16, -64, Math.toRadians(90));
    public static final Pose2d redBasket_SubDepoPos = new Pose2d(-4, -32, Math.toRadians(90));
    public static final Pose2d redBasket_MidWayPos = new Pose2d(-40,-36,Math.toRadians(90));
    public static final Pose2d redBasket_ParkPos = new Pose2d(-20,-10,Math.toRadians(180));

    public static final Pose2d redBasket_ParkHangPos = new Pose2d(-18,-10,Math.toRadians(180));
    public static final Pose2d redBasket_BasketDrop = new Pose2d(-56,-53, Math.toRadians(45));
    public static final Pose2d redBasket_RightSampleZonePos =new Pose2d(-48, -48, Math.toRadians(90));
    public static final Pose2d redBasket_MidSampleZonePos = new Pose2d(-60,-48, Math.toRadians(90));
    public static final Pose2d redBasket_LeftSampleZonePos = new Pose2d(-62,-48, Math.toRadians(130));
    public static final Pose2d redBasket_AscentPos = new Pose2d(-20, -8, Math.toRadians(180));


//Actions

   //Red

    //Red Spec Actions
    public static Action redSpec_StartToSub;
    public static Action redSpec_SubToMidWay;
    public static Action redSpec_LeftSpecToObs;
    public static Action redSpec_MidWayToLeftSpec;
    public static Action redSpec_ObsToMidSpec;
    public static Action redSpec_MidSpecToObs;
    public static Action redSpec_ObsToRightSpec;
    public static Action redSpec_SpecDepoToObs;
    public static Action redSpec_RightSpecToObs;
    public static Action redSpec_RightSpecToSub;
    public static Action redSpec_ObsToSub;
    public static Action redSpec_SubToObs;
    public static Action redSpec_ObsSpecCheck;
//    public static Action redSpec_Park;




    //Red Basket Actions
    public static Action redBasket_StartToSub;
    public static Action redBasket_SubToRightSample;
    public static Action redBasket_SubToMidWayLeftSpec;
    public static Action redBasket_MidWayToPark;
    public static Action redBasket_RightSampleToBasket;
    public static Action redBasket_ToMidSample;
    public static Action redBasket_MidSampleToBasket;
    public static Action redBasket_BasketToLeftSample;
    public static Action redBasket_LeftSampleToBasket;
    public static Action redBasket_BasketToAscent;

    public static Action  redBasket_ParkToHang;

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

        //red specimen auto

        redSpec_StartToSub =
                drive.actionBuilder(redSpec_StartPos)
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(redSpec_SubDepoPos, Math.toRadians(90))
                        .build();


        redSpec_SubToMidWay =
                drive.actionBuilder(redSpec_SubDepoPos)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(redSpec_MidWayPos,Math.toRadians(90))
                        .build();


        redSpec_MidWayToLeftSpec =
                drive.actionBuilder(redSpec_MidWayPos)
                        .setTangent(Math.toRadians(90))
                        //.splineToLinearHeading(redSpec_LeftSpecZonePos, Math.toRadians(0))
                        .splineToLinearHeading(redSpec_ObsSpecPos, Math.toRadians(0))
                        .build();

        redSpec_LeftSpecToObs =
                drive.actionBuilder(redSpec_LeftSpecZonePos)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(redSpec_LeftSpecDepoPos, Math.toRadians(270))
                        .build();

        redSpec_ObsToMidSpec =
                drive.actionBuilder(redSpec_LeftSpecDepoPos)
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(redSpec_MidSpecZonePos, Math.toRadians(0))
                        .build();

        redSpec_MidSpecToObs =
                drive.actionBuilder(redSpec_MidSpecZonePos)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(redSpec_MidSpecDepoPos, Math.toRadians(270))
                        .build();

        redSpec_ObsToRightSpec =
                drive.actionBuilder(redSpec_MidSpecDepoPos)
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(redSpec_RightSpecZonePos, Math.toRadians(0))
                        .build();

        redSpec_RightSpecToObs =
                drive.actionBuilder(redSpec_RightSpecZonePos)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(redSpec_RightSpecDepoPos, Math.toRadians(270))
                        .build();

        redSpec_RightSpecToSub =
                drive.actionBuilder(redSpec_RightSpecDepoPos)
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(redSpec_SubDepoPos, Math.toRadians(90))
                        .build();

        redSpec_SubToObs =
                drive.actionBuilder(redSpec_SubDepoPos)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(redSpec_ObsSpecPos, Math.toRadians(180))
                        .build();

        redSpec_ObsToSub =
                drive.actionBuilder(redSpec_ObsSpecPos)
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(redSpec_SubDepoPos, Math.toRadians(90))
                        .build();



//        redSpec_SpecDepoToObs =
//                drive.actionBuilder(redSpec_RightSpecDepoPos)
//                        .setTangent(180)
//                        .splineToLinearHeading(redSpec_ObsSpecPos, Math.toRadians(180))
//
//                        .build();
//
//        redSpec_ObsSpecCheck =
//                drive.actionBuilder(redSpec_ObsSpecPos)
//                        .setTangent(0)
//                        .splineToLinearHeading(redSpec_ObsSpecCheckPos, Math.toRadians(0))
//                        .build();

//        redSpec_Park =
//                drive.actionBuilder(redSpec_SubDepoPos)
//                        .setTangent(180)
//                        .splineToLinearHeading(redSpec_ObsParkPos, Math.toRadians(270))
//                        .build();

//        redSpec_RightSpecToObs =
//                drive.actionBuilder(redSpec_RightSpecZonePos)
//                        .setTangent(180)
//                        .splineToLinearHeading(redSpec_ObsSpecPos, Math.toRadians(270))
//                        .build();
//        redSpec_LeftSpecToObs =
//                drive.actionBuilder(redSpec_RightSpecZonePos)
//                        .setTangent(180)
//                        .splineToLinearHeading(redSpec_ObsSpecPos, Math.toRadians(90))
//                        .build();
//
//        redSpec_MidSpecToObs =
//                drive.actionBuilder(redSpec_MidSpecZonePos)
//                        .setTangent(0)
//                        .splineToLinearHeading(redSpec_ObsSpecPos, Math.toRadians(0))
//                        .build();



        //red basket auto

        redBasket_StartToSub =
                drive.actionBuilder( redBasket_StartPos)
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(redSpec_SubDepoPos, Math.toRadians(90))
                        .build();

        redBasket_SubToRightSample =
                drive.actionBuilder(redBasket_SubDepoPos)
                        .setTangent(270)
                        .splineToLinearHeading(redBasket_RightSampleZonePos, Math.toRadians(180))
                        .build();

        redBasket_ParkToHang =
                drive.actionBuilder(redBasket_ParkPos)
                        .setTangent(Math.toRadians(90))
                        //.splineToLinearHeading(redSpec_LeftSpecZonePos, Math.toRadians(0))
                        .splineToLinearHeading(redBasket_ParkHangPos, Math.toRadians(0))
                        .build();

        redBasket_SubToMidWayLeftSpec =
                drive.actionBuilder(redBasket_SubDepoPos)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(redBasket_MidWayPos,Math.toRadians(90))
                        .build();

        redBasket_MidWayToPark =
                drive.actionBuilder(redBasket_MidWayPos)
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(redBasket_ParkPos,Math.toRadians(315))
                        .build();

        redBasket_RightSampleToBasket =
                drive.actionBuilder(redBasket_RightSampleZonePos)
                        .setTangent(180)
                        .splineToLinearHeading(redBasket_BasketDrop, Math.toRadians(225))
                        .build();

        redBasket_ToMidSample =
                drive.actionBuilder(redBasket_BasketDrop)
                        .setTangent(90)
                        .splineToLinearHeading(redBasket_MidSampleZonePos, Math.toRadians(90))
                        .build();

        redBasket_MidSampleToBasket =
                drive.actionBuilder(redBasket_MidSampleZonePos)
                        .setTangent(180)
                        .splineToLinearHeading(redBasket_BasketDrop, Math.toRadians(225))
                        .build();

        redBasket_BasketToLeftSample =
                drive.actionBuilder(redBasket_BasketDrop)
                        .setTangent(90)
                        .splineToLinearHeading(redBasket_LeftSampleZonePos, Math.toRadians(90))
                        .build();

        redBasket_LeftSampleToBasket =
                drive.actionBuilder(redBasket_LeftSampleZonePos)
                        .setTangent(180)
                        .splineToLinearHeading(redBasket_BasketDrop, Math.toRadians(270))
                        .build();

        redBasket_BasketToAscent =
                drive.actionBuilder(redBasket_BasketDrop)
                        .setTangent(90)
                        .splineToLinearHeading(redBasket_AscentPos, Math.toRadians(0))
                        .build();

    }}