package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static final Pose2d redBasket_StartPos = new Pose2d(-16, -64, Math.toRadians(90));
    public static final Pose2d redBasket_SubDepoPos = new Pose2d(-2, -32, Math.toRadians(90));
    public static final Pose2d redBasket_BasketDrop = new Pose2d(-56,-53, Math.toRadians(45)); //?
    public static final Pose2d redBasket_RightSampleZonePos =new Pose2d(-48, -48, Math.toRadians(90));
    public static final Pose2d redBasket_MidSampleZonePos = new Pose2d(-60,-48, Math.toRadians(90));
    public static final Pose2d redBasket_LeftSampleZonePos = new Pose2d(-62,-48, Math.toRadians(130));
    public static final Pose2d redBasket_AscentParkPos = new Pose2d(-22, -8, Math.toRadians(180));



    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(360), Math.toRadians(360), 15)
                .build();


        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-16, -64, Math.toRadians(90)))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(redBasket_SubDepoPos, Math.toRadians(90))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(redBasket_RightSampleZonePos, Math.toRadians(180))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(redBasket_BasketDrop, Math.toRadians(225))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(redBasket_MidSampleZonePos, Math.toRadians(90))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(redBasket_BasketDrop, Math.toRadians(225))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(redBasket_LeftSampleZonePos, Math.toRadians(90))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(redBasket_BasketDrop, Math.toRadians(270))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(redBasket_AscentParkPos, Math.toRadians(0))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}