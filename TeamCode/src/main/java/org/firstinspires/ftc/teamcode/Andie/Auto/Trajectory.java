package org.firstinspires.ftc.teamcode.Andie.Auto;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.TestBed.Tuning.MecanumDrive;

import java.util.Set;


public class Trajectory {
    //    Basket Poses

    //Red
    public static final Pose2d redBasket_StartPos = new Pose2d(-15, -64, Math.toRadians(270));
    public static final Pose2d redBasket_BasketDrop = new Pose2d(-56,-53, Math.toRadians(45));
    public static final Pose2d redBasket_SubDrop = new Pose2d(-10, -36, Math.toRadians(270));
    public static final Pose2d redBasket_RightSampleZonePos =new Pose2d(-45, -40, Math.toRadians(90));
    public static final Pose2d redBasket_MidSampleZonePos = new Pose2d(-55,-40, Math.toRadians(90));
    public static final Pose2d redBasket_LeftSampleZonePos = new Pose2d(-53,-40, Math.toRadians(130));
    public static final Pose2d redBasket_AscentPos = new Pose2d(-28, -11, Math.toRadians(180));

    //Blue
//    public static final Pose2d blueBasket_StartPos = new Pose2d(-15, 64, Math.toRadians(270));
//    public static final Pose2d blueBasket_BasketDrop = new Pose2d(-56,53, Math.toRadians(45));
//    public static final Pose2d blueBasket_SubDrop = new Pose2d(-10, 36, Math.toRadians(270));
//    public static final Pose2d blueBasket_RightSampleZonePos =new Pose2d(-45, 40, Math.toRadians(90));
//    public static final Pose2d blueBasket_MidSampleZonePos = new Pose2d(-55,40, Math.toRadians(90));
//    public static final Pose2d blueBasket_LeftSampleZonePos = new Pose2d(-53,40, Math.toRadians(130));
//    public static final Pose2d blueBasket_AscentPos = new Pose2d(-28, 11, Math.toRadians(180));

    //Specimen Poses

    //Red
    public static final Pose2d redSpec_StartPos = new Pose2d(15, -64, Math.toRadians(270));
    public static final Pose2d redSpec_SubDepoPos = new Pose2d(10, -36, Math.toRadians(270));
    public static final Pose2d redSpec_ObsSpecPos = new Pose2d(33, -63, Math.toRadians(270));
    public static final Pose2d redSpec_RightSampleZonePos = new Pose2d(58, -45, Math.toRadians(75));
    public static final Pose2d redSpec_MidSampleZonePos = new Pose2d(58, -45, Math.toRadians(90));
    public static final Pose2d redSpec_LeftSampleZonePos = new Pose2d(50, -45, Math.toRadians(90));

    //Blue
//    public static final Pose2d blueSpec_StartPos = new Pose2d(15, 64, Math.toRadians(270));
//    public static final Pose2d blueSpec_SubDepoPos = new Pose2d(10, 36, Math.toRadians(270));
//    public static final Pose2d blueSpec_ObsSpecPos = new Pose2d(33, 63, Math.toRadians(270));
//    public static final Pose2d blueSpec_RightSampleZonePos = new Pose2d(58, 45, Math.toRadians(75));
//    public static final Pose2d blueSpec_MidSampleZonePos = new Pose2d(58, 45, Math.toRadians(90));
//    public static final Pose2d blueSpec_LeftSampleZonePos = new Pose2d(50, 45, Math.toRadians(90));

    public static Action RedSpecimenAuto;
    public static Action RedBasketAuto;
//    public static Action BlueSpecimenAuto;
//    public static Action BlueBasketAuto;

    public static Set<Subsystem> Drive;



    public static void generateTrajectories(MecanumDrive drive) {

        RedSpecimenAuto =
                drive.actionBuilder(redSpec_StartPos)
                        .setTangent(90)
                        .splineToLinearHeading(redSpec_SubDepoPos, Math.toRadians(90))
                        .waitSeconds(2) //Drop off Specimen at Submersible
                        .setTangent(200)
                        .splineToLinearHeading(redSpec_LeftSampleZonePos, Math.toRadians(0))
                        .waitSeconds(2) //Retrieve Left Sample Zone and deposit in Obs Zone
                        .setTangent(0)
                        .splineToLinearHeading(redSpec_MidSampleZonePos, Math.toRadians(0))
                        .waitSeconds(2) //Retrieve Middle Sample Zone and deposit in Obs Zone
                        .setTangent(270)
                        .splineToLinearHeading(redSpec_RightSampleZonePos, Math.toRadians(0))
                        .waitSeconds(2) //Retrieve Right Sample Zone and deposit in Obs Zone
                        .setTangent(180)
                        .splineToLinearHeading(redSpec_ObsSpecPos, Math.toRadians(270))
                        .waitSeconds(2) //Retrieve Specimen from Obs Zone
                        .setTangent(90)
                        .splineToLinearHeading(redSpec_SubDepoPos, Math.toRadians(90))
                        .waitSeconds(2) //Drop off Specimen at Submersible
                        .setTangent(135)
                        .splineToLinearHeading(redSpec_ObsSpecPos, Math.toRadians(180))
                        .waitSeconds(2) //Retrieve Specimen from Obs Zone
                        .setTangent(90)
                        .splineToLinearHeading(redSpec_SubDepoPos, Math.toRadians(90))
                        .waitSeconds(2) //Drop off Specimen at Submersible
                        .setTangent(200)
                        .splineToLinearHeading(redSpec_ObsSpecPos, Math.toRadians(270))
                        .waitSeconds(2) //Pick up Specimen from Obs Zone
                        .setTangent(90)
                        .splineToLinearHeading(redSpec_SubDepoPos, Math.toRadians(90))
                        .build();

        RedBasketAuto =
                drive.actionBuilder(redBasket_StartPos)
                        .setTangent(70)
                        .splineToLinearHeading(redBasket_SubDrop, Math.toRadians(90))
                        .setTangent(180)
                        .splineToLinearHeading(redBasket_RightSampleZonePos, Math.toRadians(180))
                        .waitSeconds(1)
                        .setTangent(180)
                        .splineToLinearHeading(redBasket_BasketDrop, Math.toRadians(270))
                        .waitSeconds(2)
                        .setTangent(45)
                        .splineToLinearHeading(redBasket_MidSampleZonePos, Math.toRadians(90))
                        .waitSeconds(1)
                        .setTangent(180)
                        .splineToLinearHeading(redBasket_BasketDrop, Math.toRadians(270))
                        .waitSeconds(2)
                        .setTangent(45)
                        .splineToLinearHeading(redBasket_LeftSampleZonePos, Math.toRadians(90))
                        .waitSeconds(1)
                        .setTangent(180)
                        .splineToLinearHeading(redBasket_BasketDrop, Math.toRadians(270))
                        .waitSeconds(2)
                        .setTangent(0)
                        .splineToLinearHeading(redBasket_AscentPos, Math.toRadians(0))
                        .build();
    }

}

