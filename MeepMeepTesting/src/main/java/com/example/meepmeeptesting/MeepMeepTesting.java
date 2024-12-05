package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
//    public static final Pose2d redSpec_StartPos = new Pose2d(8, -62, Math.toRadians(90));
//    public static final Pose2d redSpec_SubDepoPos = new Pose2d(36, -34, Math.toRadians(90));
//    public static final Pose2d redSpec_ObsSpecPos = new Pose2d(36, -61, Math.toRadians(90));
//    public static final Pose2d redSpec_ObsDepoPos = new Pose2d(36, -52, Math.toRadians(330));
//    public static final Pose2d redSpec_RightSpecZonePos = new Pose2d(36, -48, Math.toRadians(30));
//    public static final Pose2d redSpec_MidSpecZonePos = new Pose2d(36, -48, Math.toRadians(45));
//    public static final Pose2d redSpec_LeftSpecZonePos = new Pose2d(36, -48, Math.toRadians(60));
//    public static final Pose2d redSpec_ObsParkPos = new Pose2d(36,-63,Math.toRadians(90));


    // auto 1 (ddf75d7)
    public static final Pose2d redSpec_StartPos = new Pose2d(16, -64, Math.toRadians(90));
    //   public static final Pose2d redSpec_StartPos = new Pose2d(40, -64, Math.toRadians(90));
    public static final Pose2d redSpec_SubDepoPos = new Pose2d(4, -32, Math.toRadians(90));
    public static final Pose2d redSpec_ObsSpecPos = new Pose2d(42, -61, Math.toRadians(90));
    public static final Pose2d redSpec_ObsSpecCheckPos = new Pose2d(48,-61,Math.toRadians(90));
    public static final Pose2d redSpec_LeftSpecZonePos = new Pose2d(46, -15, Math.toRadians(90));
    public static final Pose2d redSpec_LeftSpecDepoPos = new Pose2d(46, -56, Math.toRadians(90));
    public static final Pose2d redSpec_MidSpecZonePos = new Pose2d(60, -14, Math.toRadians(90));
    public static final Pose2d redSpec_MidSpecDepoPos = new Pose2d(60, -56, Math.toRadians(90));
    public static final Pose2d redSpec_RightSpecZonePos = new Pose2d(67, -14, Math.toRadians(90));
    public static final Pose2d redSpec_RightSpecDepoPos = new Pose2d(67, -66, Math.toRadians(90));
    public static final Pose2d redSpec_MidWayPos = new Pose2d(36,-40,Math.toRadians(90));

    // auto 2 (310337b)
//    public static final Pose2d redSpec_StartPos = new Pose2d(16, -64, Math.toRadians(90));
//    //   public static final Pose2d redSpec_StartPos = new Pose2d(40, -64, Math.toRadians(90));
//    public static final Pose2d redSpec_SubDepoPos = new Pose2d(3, -32, Math.toRadians(90));
//    public static final Pose2d redSpec_ObsSpecPos = new Pose2d(42, -61, Math.toRadians(90));
//    public static final Pose2d redSpec_ObsSpecCheckPos = new Pose2d(48,-61,Math.toRadians(90));
//    public static final Pose2d redSpec_LeftSpecZonePos = new Pose2d(42, -17, Math.toRadians(90));
//    public static final Pose2d redSpec_LeftSpecDepoPos = new Pose2d(42, -56, Math.toRadians(90));
//    public static final Pose2d redSpec_MidSpecZonePos = new Pose2d(52, -14, Math.toRadians(90));
//    public static final Pose2d redSpec_MidSpecDepoPos = new Pose2d(52, -56, Math.toRadians(90));
//    public static final Pose2d redSpec_RightSpecZonePos = new Pose2d(62, -14, Math.toRadians(90));
//    public static final Pose2d redSpec_RightSpecDepoPos = new Pose2d(62, -67, Math.toRadians(90));
//    public static final Pose2d redSpec_MidWayPos = new Pose2d(34,-42,Math.toRadians(90));

    // auto 3 (90369ce)
//    public static final Pose2d redSpec_StartPos = new Pose2d(16, -64, Math.toRadians(90));
//    //   public static final Pose2d redSpec_StartPos = new Pose2d(40, -64, Math.toRadians(90));
//    public static final Pose2d redSpec_SubDepoPos = new Pose2d(3, -32, Math.toRadians(90));
//    public static final Pose2d redSpec_ObsSpecPos = new Pose2d(42, -61, Math.toRadians(90));
//    public static final Pose2d redSpec_ObsSpecCheckPos = new Pose2d(48,-61,Math.toRadians(90));
//    public static final Pose2d redSpec_LeftSpecZonePos = new Pose2d(42, -17, Math.toRadians(90));
//    public static final Pose2d redSpec_LeftSpecDepoPos = new Pose2d(42, -56, Math.toRadians(90));
//    public static final Pose2d redSpec_MidSpecZonePos = new Pose2d(52, -14, Math.toRadians(90));
//    public static final Pose2d redSpec_MidSpecDepoPos = new Pose2d(52, -56, Math.toRadians(90));
//    public static final Pose2d redSpec_RightSpecZonePos = new Pose2d(62, -14, Math.toRadians(90));
//    public static final Pose2d redSpec_RightSpecDepoPos = new Pose2d(62, -67, Math.toRadians(90));
//    public static final Pose2d redSpec_MidWayPos = new Pose2d(34,-42,Math.toRadians(90));


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(360), Math.toRadians(360), 15)
                .build();

                /*
new ParallelActionCommand(arm, wrist, gripper, lift, exampleSubsystem,"redSpec_StartToSub"),

new ParallelActionCommand(arm, wrist, gripper, lift, exampleSubsystem, "redSpec_SubToLeftSpec"),

RedSpec_MidWayToLeftSpec,

RedSpec_LeftSpecToObs,

RedSpec_ObsToMidSpec,

RedSpec_MidSpecToObs,

RedSpec_ObsToRightSpec,

new ParallelActionCommand(arm, wrist, gripper, lift, exampleSubsystem, "RightSpecPickUpSpecimen")
                */

        // auto 1 (ddf75d7)
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(16, -64, Math.toRadians(90)))
                .setTangent(90)
                .splineToLinearHeading(redSpec_SubDepoPos, Math.toRadians(0))
                .setTangent(180)
                .splineToLinearHeading(redSpec_MidWayPos,Math.toRadians(90))
                .setTangent(180)
                .splineToLinearHeading(redSpec_LeftSpecZonePos, Math.toRadians(270))
                .setTangent(180)
                .splineToLinearHeading(redSpec_LeftSpecDepoPos, Math.toRadians(270))
                .setTangent(90)
                .splineToLinearHeading(redSpec_MidSpecZonePos, Math.toRadians(0))
                .setTangent(180)
                .splineToLinearHeading(redSpec_MidSpecDepoPos, Math.toRadians(0))

        // auto 2 (310337b)
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(16, -64, Math.toRadians(90)))
//                .setTangent(90)
//                .splineToLinearHeading(redSpec_SubDepoPos, Math.toRadians(90))
//                .setTangent(270)
//                .splineToLinearHeading(redSpec_MidWayPos,Math.toRadians(0))
//                .setTangent(270)
//                .splineToLinearHeading(redSpec_LeftSpecZonePos, Math.toRadians(90)) //0 angle
//                .setTangent(270)
//                .splineToLinearHeading(redSpec_LeftSpecDepoPos, Math.toRadians(270))
//                .setTangent(90)
//                .splineToLinearHeading(redSpec_MidSpecZonePos, Math.toRadians(90))
//                .setTangent(270)
//                .splineToLinearHeading(redSpec_MidSpecDepoPos, Math.toRadians(270))
//                .setTangent(90)
//                .splineToLinearHeading(redSpec_RightSpecZonePos, Math.toRadians(0))
//                .setTangent(270)
//                .splineToLinearHeading(redSpec_RightSpecDepoPos, Math.toRadians(270))

        // auto 3 (90369ce)
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(16, -64, Math.toRadians(90)))
//                .setTangent(90)
//                .splineToLinearHeading(redSpec_SubDepoPos, Math.toRadians(90))
//                .setTangent(180)
//                .splineToLinearHeading(redSpec_MidWayPos,Math.toRadians(90))
//                .setTangent(180)
//                .splineToLinearHeading(redSpec_LeftSpecZonePos, Math.toRadians(270))
//                .setTangent(270)
//                .splineToLinearHeading(redSpec_LeftSpecDepoPos, Math.toRadians(270))
//                .setTangent(90)
//                .splineToLinearHeading(redSpec_MidSpecZonePos, Math.toRadians(90))
//                .setTangent(180)
//                .splineToLinearHeading(redSpec_MidSpecDepoPos, Math.toRadians(270))
//                .setTangent(90)
//                .splineToLinearHeading(redSpec_RightSpecZonePos, Math.toRadians(0))


                /*
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(8, -62, Math.toRadians(90)))
                .setTangent(90)
                .splineToLinearHeading(redSpec_SubDepoPos, Math.toRadians(90))
                .waitSeconds(2)
                .setTangent(300)
                .splineToLinearHeading(redSpec_LeftSpecZonePos, Math.toRadians(0))
                .setTangent(0)
                .splineToLinearHeading(redSpec_ObsDepoPos, Math.toRadians(0))
                .setTangent(0)
                .splineToLinearHeading(redSpec_MidSpecZonePos, Math.toRadians(0))
                .setTangent(0)
                .splineToLinearHeading(redSpec_ObsDepoPos, Math.toRadians(0))
                .setTangent(0)
                .splineToLinearHeading(redSpec_RightSpecZonePos, Math.toRadians(0))
                .setTangent(0)
                .splineToLinearHeading(redSpec_ObsDepoPos, Math.toRadians(0))
*/


                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}