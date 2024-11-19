package org.firstinspires.ftc.teamcode.TestBed.AutoPathing;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TestBed.Tuning.MecanumDrive;


@Config
@Autonomous(name = "RedBasketAuto", group = "Autonomous")
public class RedBasketAuto extends LinearOpMode {

    //Basket Positions
    public static final Pose2d redBasket_StartPos = new Pose2d(-15, -64, Math.toRadians(270));
    public static final Pose2d redBasket_BasketDrop = new Pose2d(-56,-53, Math.toRadians(45));
    public static final Pose2d redBasket_SubDrop = new Pose2d(-10, -36, Math.toRadians(270));
    public static final Pose2d redBasket_RightSampleZonePos =new Pose2d(-45, -40, Math.toRadians(90));
    public static final Pose2d redBasket_MidSampleZonePos = new Pose2d(-55,-40, Math.toRadians(90));
    public static final Pose2d redBasket_LeftSampleZonePos = new Pose2d(-53,-40, Math.toRadians(130));
    public static final Pose2d redBasket_AscentPos = new Pose2d(-28, -11, Math.toRadians(180));
    public final Pose2d redBasket_ObsSampPos = new Pose2d(33, -63, Math.toRadians(270));


    @Override

    public void runOpMode() {
        Pose2d initialPose = redBasket_StartPos;
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // vision here that outputs position
        int visionOutputPosition = 1;



        Action TrajectoryActionBuilder = drive.actionBuilder(initialPose)
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


        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

//        Action trajectoryActionChosen;
//        trajectoryActionChosen = tab1.build();

        Actions.runBlocking(
                new SequentialAction(
                        TrajectoryActionBuilder
                )
        );
    }
}