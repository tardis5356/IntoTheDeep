package org.firstinspires.ftc.teamcode.TestBed;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TestBed.TestBedTrajectory;



@Config
@Autonomous(name = "TestBedAuto", group = "Autonomous")
public abstract class TestBedAuto extends TestBedTrajectory {
    @Override
public void runOpMode() {
    MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.8, 61.7, Math.toRadians(90)));

    // vision here that outputs position
    int visionOutputPosition = 1;

    Action trajectoryAction1;
    Action trajectoryActionCloseOut;

    trajectoryAction1 = drive.actionBuilder(drive.pose)
            .setTangent(Math.toRadians(0))
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

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryAction1)
                );
}
}



