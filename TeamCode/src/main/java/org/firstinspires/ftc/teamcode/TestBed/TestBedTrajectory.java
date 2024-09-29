package org.firstinspires.ftc.teamcode.TestBed;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;




public abstract class TestBedTrajectory {



        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.8, 61.7, Math.toRadians(90)));

        // vision here that outputs position
        int visionOutputPosition = 1;

        Action trajectoryAction1;
        Action trajectoryAction2;
        Action trajectoryAction3;
        Action trajectoryActionCloseOut;

//        public Action getTrajectoryAction1() {
//                trajectoryAction1 = drive.actionBuilder(drive.pose)
//                        .strafeTo(new Vector2d(30, -4))
//                        .build();
//
//        }
}