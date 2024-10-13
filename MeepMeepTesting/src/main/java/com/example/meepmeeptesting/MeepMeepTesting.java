package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(360), Math.toRadians(360), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(15, -64, Math.toRadians(270)))
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(10, -36, Math.toRadians(270)), Math.toRadians(90))
                .waitSeconds(2)
                .setTangent(200)
                .splineToLinearHeading(new Pose2d(50, -45, Math.toRadians(90)), Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(58, -45, Math.toRadians(90)), Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(270)
                .splineToLinearHeading(new Pose2d(58, -45, Math.toRadians(75)), Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(180)
                .splineToLinearHeading(new Pose2d(33, -63, Math.toRadians(90)), Math.toRadians(270))
                .waitSeconds(2)
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(10, -36, Math.toRadians(270)), Math.toRadians(90))
                .waitSeconds(2)
                .setTangent(200)
                .splineToLinearHeading(new Pose2d(33, -63, Math.toRadians(90)), Math.toRadians(270))
                .waitSeconds(2)
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(10, -36, Math.toRadians(270)), Math.toRadians(90))
                .waitSeconds(2)
                .setTangent(200)
                .splineToLinearHeading(new Pose2d(33, -63, Math.toRadians(90)), Math.toRadians(270))
                .waitSeconds(2)
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(10, -36, Math.toRadians(270)), Math.toRadians(90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}