package org.firstinspires.ftc.teamcode.TestBed;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.TestBed.Tuning.MecanumDrive;

import java.util.Set;


public class AutoTrajectories {

    public static final Pose2d redSpec_StartPos = new Pose2d(15, -64, Math.toRadians(270));
    public static final Pose2d redSpec_SubDepoPos = new Pose2d(10, -36, Math.toRadians(270));
    public static final Pose2d redSpec_ObsSpecPos = new Pose2d(33, -63, Math.toRadians(270));
    public static final Pose2d redSpec_RightSampleZonePos = new Pose2d(58, -45, Math.toRadians(75));
    public static final Pose2d redSpec_MidSampleZonePos = new Pose2d(58, -45, Math.toRadians(90));
    public static final Pose2d redSpec_LeftSampleZonePos = new Pose2d(50, -45, Math.toRadians(90));
    public static Action RedSpecimenAuto;
    public static Set<Subsystem> Drive;

    public static Pose2d initialPose = redSpec_StartPos;
    MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

    public static void generateTrajectories(MecanumDrive drive) {

         RedSpecimenAuto =
                drive.actionBuilder(initialPose)
                        .setTangent(90)
                        .splineToLinearHeading(redSpec_SubDepoPos, Math.toRadians(90))
                        .waitSeconds(2) //Drop off Specimen at Submersible
                        .setTangent(200)
                        .splineToLinearHeading(redSpec_LeftSampleZonePos, Math.toRadians(0))
                        .waitSeconds(2) //Retrieve Left Sample Zone and deposit in Obs Zone
                        .setTangent(0)
                        .splineToLinearHeading(redSpec_MidSampleZonePos, Math.toRadians(0))
                        .waitSeconds(2) //Retrieve Middle Sample Zone and deposit in Obs Zone
                        .setTangent(270)
                        .splineToLinearHeading(redSpec_RightSampleZonePos, Math.toRadians(0))
                        .waitSeconds(2) //Retrieve Right Sample Zone and deposit in Obs Zone
                        .setTangent(180)
                        .splineToLinearHeading(redSpec_ObsSpecPos, Math.toRadians(270))
                        .waitSeconds(2) //Retrieve Specimen from Obs Zone
                        .setTangent(90)
                        .splineToLinearHeading(redSpec_SubDepoPos, Math.toRadians(90))
                        .waitSeconds(2) //Drop off Specimen at Submersible
                        .setTangent(135)
                        .splineToLinearHeading(redSpec_ObsSpecPos, Math.toRadians(180))
                        .waitSeconds(2) //Retrieve Specimen from Obs Zone
                        .setTangent(90)
                        .splineToLinearHeading(redSpec_SubDepoPos, Math.toRadians(90))
                        .waitSeconds(2) //Drop off Specimen at Submersible
                        .setTangent(200)
                        .splineToLinearHeading(redSpec_ObsSpecPos, Math.toRadians(270))
                        .waitSeconds(2) //Pick up Specimen from Obs Zone
                        .setTangent(90)
                        .splineToLinearHeading(redSpec_SubDepoPos, Math.toRadians(90))
                        .build();
    }

}
