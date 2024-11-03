package org.firstinspires.ftc.teamcode.TestBed;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TestBed.Tuning.MecanumDrive;


@Config
@Autonomous(name = "RedSpecimenAuto", group = "Autonomous")
public class RedSpecimenAuto extends LinearOpMode {


    public final Pose2d redSpec_StartPos = new Pose2d(15, -64, Math.toRadians(270));
    public final Pose2d redSpec_SubDepoPos = new Pose2d(10, -36, Math.toRadians(270));
    public final Pose2d redSpec_ObsSpecPos = new Pose2d(33, -63, Math.toRadians(270));
    public final Pose2d redSpec_RightSampleZonePos = new Pose2d(58, -45, Math.toRadians(75));
    public final Pose2d redSpec_MidSampleZonePos = new Pose2d(58, -45, Math.toRadians(90));
    public final Pose2d redSpec_LeftSampleZonePos = new Pose2d(50, -45, Math.toRadians(90));





    @Override

    public void runOpMode() {
        MultipleTelemetry telemetry2 = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d initialPose = redSpec_StartPos;
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // vision here that outputs position
        int visionOutputPosition = 1;

        telemetry.setMsTransmissionInterval(50);

        telemetry.addData("Heading",Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.addData("X",drive.pose.position.x);
        telemetry.addData("Y",drive.pose.position.y);
        telemetry.update();



        telemetry.addData("Heading",Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.addData("X",drive.pose.position.x);
        telemetry.addData("Y",drive.pose.position.y);
        telemetry.update();


        Action RedSpecAuto = drive.actionBuilder(initialPose)
                .waitSeconds(20)
                .setTangent(90)
                .splineToLinearHeading(redSpec_SubDepoPos, Math.toRadians(90))
                .waitSeconds(2) //Drop off Specimen at Submersible
                .setTangent(200)
                .splineToLinearHeading(redSpec_LeftSampleZonePos, Math.toRadians(0))
                .waitSeconds(2) //Retrieve Left Sample Zone and deposit in Obs Zone
//                .setTangent(0)
//                .splineToLinearHeading(redSpec_MidSampleZonePos, Math.toRadians(0))
//                .waitSeconds(2) //Retrieve Middle Sample Zone and deposit in Obs Zone
//                .setTangent(270)
//                .splineToLinearHeading(redSpec_RightSampleZonePos, Math.toRadians(0))
//                .waitSeconds(2) //Retrieve Right Sample Zone and deposit in Obs Zone
//                .setTangent(180)
//                .splineToLinearHeading(redSpec_ObsSpecPos, Math.toRadians(270))
//                .waitSeconds(2) //Retrieve Specimen from Obs Zone
//                .setTangent(90)
//                .splineToLinearHeading(redSpec_SubDepoPos, Math.toRadians(90))
//                .waitSeconds(2) //Drop off Specimen at Submersible
//                .setTangent(135)
//                .splineToLinearHeading(redSpec_ObsSpecPos, Math.toRadians(180))
//                .waitSeconds(2) //Retrieve Specimen from Obs Zone
//                .setTangent(90)
//                .splineToLinearHeading(redSpec_SubDepoPos, Math.toRadians(90))
//                .waitSeconds(2) //Drop off Specimen at Submersible
//                .setTangent(200)
//                .splineToLinearHeading(redSpec_ObsSpecPos, Math.toRadians(270))
//                .waitSeconds(2) //Pick up Specimen from Obs Zone
//                .setTangent(90)
//                .splineToLinearHeading(redSpec_SubDepoPos, Math.toRadians(90))
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
//        while (opModeIsActive()) {



        Actions.runBlocking(
                RedSpecAuto

        );


    }
}
