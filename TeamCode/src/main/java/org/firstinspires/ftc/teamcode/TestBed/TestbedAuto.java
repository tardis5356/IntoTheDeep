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
@Autonomous(name = "TestBedAuto", group = "Autonomous")
public class TestbedAuto extends LinearOpMode {
    @Override

    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-15, -64, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // vision here that outputs position
        int visionOutputPosition = 1;



         Action TrajectoryActionBuilder = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-10, -36))
                .waitSeconds(1)
                .strafeTo(new Vector2d(48,-36))
                .build();

        // .strafeTo(new Vector2d(36, -4));
        //.lineToX(30)
        //.lineToY(-4)
        //Action trajectoryActionCloseOut = tab1.fresh()
//                .strafeTo(new Vector2d(30, 32))
//                .waitSeconds(1)
//                .strafeTo(new Vector2d(32,-36))
//                .build();



//    trajectoryAction2 = drive.actionBuilder(drive.pose)
//            .turn(Math.toRadians(90))
//            .lineToY(-48)
//            .lineToX(48)
//            .build();

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
