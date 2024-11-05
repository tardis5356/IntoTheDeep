package org.firstinspires.ftc.teamcode.TestBed;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TestBed.Tuning.MecanumDrive;


@Config
@Autonomous(name = "RedSpecimenAuto", group = "Autonomous")
public class RedSpecimenAuto extends LinearOpMode {

    public final Pose2d redSpec_StartPos = new Pose2d(15, -64, Math.toRadians(270));
    public final Pose2d redSpec_SubDepoPos = new Pose2d(6, -36, Math.toRadians(270));
    public final Pose2d redSpec_ObsSpecPos = new Pose2d(38, -63, Math.toRadians(270));
    public final Pose2d redSpec_RightSampleZonePos = new Pose2d(60, -10, Math.toRadians(89.999999));
    public final Pose2d redSpec_MidSampleZonePos = new Pose2d(58, -42, Math.toRadians(89.999999)); //450 or 90
    public final Pose2d redSpec_LeftSampleZonePos = new Pose2d(50, -42, Math.toRadians(450)); // 450 or 90
    public final Pose2d redSpec_ObsPark = new Pose2d(40,-60,Math.toRadians(269.999999));


    @Override

    public void runOpMode() {
        Pose2d initialPose = redSpec_StartPos;
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // vision here that outputs position
        int visionOutputPosition = 1;


        Action RedSpecAuto = drive.actionBuilder(initialPose)

                .setTangent(90)
                .splineToLinearHeading(redSpec_SubDepoPos, Math.toRadians(90))
                .waitSeconds(2) //Drop off Specimen at Submersible - counterclockwise 90 degrees

                .setTangent(270)
                .splineToLinearHeading(redSpec_LeftSampleZonePos, Math.toRadians(0))
                .waitSeconds(2) //Retrieve Left Sample Zone and deposit in Obs Zone

                .setTangent(0)
                .splineToLinearHeading(redSpec_MidSampleZonePos, Math.toRadians(0))
                .waitSeconds(2) //Retrieve Middle Sample Zone and deposit in Obs Zone

                .setTangent(180)
                .splineToLinearHeading(redSpec_ObsSpecPos, Math.toRadians(270))
                .waitSeconds(2) //Retrieve Specimen from Obs Zone

                .setTangent(180)
                .splineToLinearHeading(redSpec_SubDepoPos, Math.toRadians(90))
                .waitSeconds(2) //Drop off Specimen at Submersible

                .setTangent(0)
                .lineToXConstantHeading(50)
                .splineToLinearHeading(redSpec_RightSampleZonePos, Math.toRadians(90))
                .lineToYConstantHeading(-56)
                .waitSeconds(2)//Retrieve Right Sample Zone and deposit in Obs Zone

                .setTangent(180)
                .splineToLinearHeading(redSpec_ObsSpecPos, Math.toRadians(270))
                .waitSeconds(2) //Retrieve Specimen from Obs Zone

                .setTangent(90)
                .splineToLinearHeading(redSpec_SubDepoPos, Math.toRadians(90))
                .waitSeconds(2) //Drop off Specimen at Submersible

                .setTangent(300)
                .splineToLinearHeading(redSpec_ObsSpecPos, Math.toRadians(270))
                .waitSeconds(2) //Pick up Specimen from Obs Zone

                .setTangent(130)
                .splineToLinearHeading(redSpec_SubDepoPos, Math.toRadians(90))
                .waitSeconds(1) //Drop off Specimen at Submersible

                .setTangent(270)
                .splineToLinearHeading(redSpec_ObsPark,Math.toRadians(270))//Observation zone park

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
                        RedSpecAuto
                )
        );
    }
}
