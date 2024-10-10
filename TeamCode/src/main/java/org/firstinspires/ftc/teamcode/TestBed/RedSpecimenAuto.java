package org.firstinspires.ftc.teamcode.TestBed;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;




@Config
@Autonomous(name = "RedSpecimenAuto", group = "Autonomous")
public class RedSpecimenAuto extends LinearOpMode {
    @Override

    public void runOpMode() {
        Pose2d initialPose = new Pose2d(15, -64, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // vision here that outputs position
        int visionOutputPosition = 1;



        Action TrajectoryActionBuilder = drive.actionBuilder(initialPose)
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(10,-36, Math.toRadians(270)), Math.PI / 2)
                .waitSeconds(2)
                .setTangent(270)
                .splineToLinearHeading(new Pose2d(50,-52,Math.toRadians(90)),Math.PI/2)
                .waitSeconds(2)
                .setTangent(90)
                .strafeTo(new Vector2d(56,-52))
                .waitSeconds(2)
                .setTangent(90)
                .strafeTo(new Vector2d(33,-63))
                .splineToLinearHeading(new Pose2d(10,-36,Math.toRadians(270)),Math.PI/2)
                .waitSeconds(2)
                .strafeTo(new Vector2d(38,-36))
                .splineToLinearHeading(new Pose2d(62,-12,Math.toRadians(270)),Math.PI/2)
                .waitSeconds(2)
                .strafeTo(new Vector2d(64,-62))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(10,-36, Math.toRadians(270)), Math.PI / 2)
                .waitSeconds(2)
                .strafeTo(new Vector2d(38,-36))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(10,-36, Math.toRadians(270)), Math.PI / 2)
                .waitSeconds(2)
                .strafeTo(new Vector2d(40,-65))
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
