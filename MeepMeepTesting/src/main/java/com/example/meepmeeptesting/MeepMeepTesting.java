package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static final Pose2d redBasket_StartPos = new Pose2d(-15, -64, Math.toRadians(270));
    public static final Pose2d redBasket_BasketDrop = new Pose2d(-53,-56, Math.toRadians(45));
    public static final Pose2d redBasket_SubDrop = new Pose2d(-10, -36, Math.toRadians(270));
    public static final Pose2d redBasket_RightSampleZonePos =new Pose2d(-48, -40, Math.toRadians(90));
    public static final Pose2d redBasket_MidSampleZonePos = new Pose2d(-57,-40, Math.toRadians(90));
    public static final Pose2d redBasket_LeftSampleZonePos = new Pose2d(-57,-40, Math.toRadians(135));
    public static final Pose2d redBasket_AscentPos = new Pose2d(-28, -11, Math.toRadians(180));
    public static final Pose2d redBasket_ObsSampPos = new Pose2d(33, -63, Math.toRadians(270));

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(360), Math.toRadians(360), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-15, -64, Math.toRadians(270)))
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
                .setTangent(90)
                .splineToLinearHeading(redBasket_ObsSampPos, Math.toRadians(270))
                .setTangent(270)
                .splineToLinearHeading(redBasket_BasketDrop, Math.toRadians(90))
                .setTangent(180)
                .splineToLinearHeading(redBasket_AscentPos, Math.toRadians(180))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}