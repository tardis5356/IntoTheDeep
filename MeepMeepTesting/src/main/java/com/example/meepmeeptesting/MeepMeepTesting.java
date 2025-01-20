package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static final Pose2d redSpec_StartPos = new Pose2d(16, -64, Math.toRadians(90));
    public static final Pose2d redSpec_SubDepoPos = new Pose2d(-3, -31, Math.toRadians(90)); //x=0
    public static final Pose2d redSpec_ObsSpecPos = new Pose2d(38, -66.25, Math.toRadians(90));
     public static final Pose2d redSpec_MidPointPos = new Pose2d(36, -40, Math.toRadians(90));
    public static final Pose2d redSpec_SubDepoPos1 = new Pose2d(0, -31, Math.toRadians(90)); //x=3
    public static final Pose2d redSpec_SubDepoPos2 = new Pose2d(3, -31, Math.toRadians(90)); //x=6
    public static final Pose2d redSpec_SubDepoPos3 = new Pose2d(6, -31, Math.toRadians(90)); //x=9
    public static final Pose2d redSpec_SubDepoPos4 = new Pose2d(9,-31, Math.toRadians(90));
    public static final Pose2d redSpecEx_LeftSpecZonePos = new Pose2d(28, -45, Math.toRadians(45));
    public static final Pose2d redSpecEx_LeftSpecDepoPos = new Pose2d(28.1, -45, Math.toRadians(340));
    public static final Pose2d redSpecEx_MidSpecZonePos = new Pose2d(40, -48, Math.toRadians(58));
    public static final Pose2d redSpecEx_MidSpecDepoPos = new Pose2d(40.8, -52, Math.toRadians(320));//320
    public static final Pose2d redSpecEx_RightSpecZonePos = new Pose2d(48, -48, Math.toRadians(55));
    public static final Pose2d redSpecEx_RightSpecDepoPos = new Pose2d(41, -57, Math.toRadians(345)); //37,-58,335
    public static final Pose2d redSpecEx_ObsPrepPos = new Pose2d(38, -60, Math.toRadians(90));

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(360), Math.toRadians(360), 15)
                .build();


        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(16, -64, Math.toRadians(90)))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(redSpec_SubDepoPos, Math.toRadians(90))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(redSpecEx_LeftSpecZonePos, Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(redSpecEx_LeftSpecDepoPos, Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(redSpecEx_MidSpecZonePos, Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(redSpecEx_MidSpecDepoPos, Math.toRadians(270))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(redSpecEx_RightSpecZonePos, Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(redSpecEx_RightSpecDepoPos, Math.toRadians(270))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(redSpecEx_ObsPrepPos, Math.toRadians(90))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(redSpec_ObsSpecPos, Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(redSpec_SubDepoPos1, Math.toRadians(90))
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(redSpec_ObsSpecPos, Math.toRadians(270))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(redSpec_SubDepoPos2, Math.toRadians(90))
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(redSpec_ObsSpecPos, Math.toRadians(270))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(redSpec_SubDepoPos3, Math.toRadians(90))
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(redSpec_ObsSpecPos, Math.toRadians(270))
                .setTangent(90)
                .splineToLinearHeading(redSpec_SubDepoPos4, Math.toRadians(90))
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(redSpec_ObsSpecPos, Math.toRadians(270))


                //Spec auto w/o extendo
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(redSpec_SubDepoPos, Math.toRadians(90))
//                .setTangent(Math.toRadians(270))
//                .splineToLinearHeading(redSpec_MidPointPos,Math.toRadians(90))
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(redSpec_LeftSpecZonePos, Math.toRadians(0))
//                .setTangent(Math.toRadians(270))
//                .splineToLinearHeading(redSpec_LeftSpecDepoPos, Math.toRadians(270))
//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(redSpec_MidSpecZonePos, Math.toRadians(0))
//                .setTangent(Math.toRadians(270))
//                .splineToLinearHeading(redSpec_MidSpecDepoPos, Math.toRadians(270))
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(redSpec_RightSpecZonePos, Math.toRadians(0))
//                .setTangent(Math.toRadians(270))
//                .splineToLinearHeading(redSpec_RightSpecDepoPos, Math.toRadians(270))
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(redSpec_SubDepoPos, Math.toRadians(90))
//                .setTangent(Math.toRadians(270))
//                .splineToLinearHeading(redSpec_ObsSpecPos, Math.toRadians(180))
//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(redSpec_SubDepoPos, Math.toRadians(90))
                //Basket Auto
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(redBasket_SubDepoPos, Math.toRadians(90))
//                .setTangent(Math.toRadians(270))
//                .splineToLinearHeading(redBasket_RightSampleZonePos, Math.toRadians(180))
//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(redBasket_BasketDrop, Math.toRadians(225))
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(redBasket_MidSampleZonePos, Math.toRadians(90))
//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(redBasket_BasketDrop, Math.toRadians(225))
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(redBasket_LeftSampleZonePos, Math.toRadians(90))
//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(redBasket_BasketDrop, Math.toRadians(270))
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(redBasket_AscentParkPos, Math.toRadians(0))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}