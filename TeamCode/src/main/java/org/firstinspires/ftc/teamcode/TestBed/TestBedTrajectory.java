package org.firstinspires.ftc.teamcode.TestBed;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

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




public abstract class TestBedTrajectory {



        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.8, 61.7, Math.toRadians(90)));

        // vision here that outputs position
        int visionOutputPosition = 1;

        Action trajectoryAction1;
        Action trajectoryAction2;
        Action trajectoryAction3;
        Action trajectoryActionCloseOut;

        public Action getTrajectoryAction1() {
                Action = drive.actionBuilder(drive.pose);

                .setTangent(Math.toRadians(0))
                        .build();
        }
}