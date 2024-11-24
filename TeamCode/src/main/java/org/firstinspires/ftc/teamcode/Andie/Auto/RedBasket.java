package org.firstinspires.ftc.teamcode.Andie.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Andie.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Extendo;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.TestBed.Tuning.MecanumDrive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous", group = "Autonomous")

public class RedBasket extends LinearOpMode{

    //gripper
    private Gripper gripper;


    //wrist
    private Wrist wrist;

    //intake
    private Intake intake;

    private Extendo extendo;

    private Arm arm;

    private ColorSensor cI;

    private Lift lift;
    private TouchSensor lL;

    private Gamepad aparatus;

    //    Basket Poses

    //Red Basket
    public static final Pose2d redBasket_StartPos = new Pose2d(-15, -64, Math.toRadians(270));
    public static final Pose2d redBasket_BasketDrop = new Pose2d(-56,-53, Math.toRadians(45));
    public static final Pose2d redBasket_SubDrop = new Pose2d(-10, -36, Math.toRadians(270));
    public static final Pose2d redBasket_RightSampleZonePos =new Pose2d(-45, -40, Math.toRadians(90));
    public static final Pose2d redBasket_MidSampleZonePos = new Pose2d(-55,-40, Math.toRadians(90));
    public static final Pose2d redBasket_LeftSampleZonePos = new Pose2d(-53,-40, Math.toRadians(130));
    public static final Pose2d redBasket_AscentPos = new Pose2d(-28, -11, Math.toRadians(180));

    //Bluebasket


    //Specimen Poses

    //Red Specimen
    public static final Pose2d redSpec_StartPos = new Pose2d(15, -64, Math.toRadians(270));
    public static final Pose2d redSpec_SubDepoPos = new Pose2d(10, -36, Math.toRadians(270));
    public static final Pose2d redSpec_ObsSpecPos = new Pose2d(33, -63, Math.toRadians(270));
    public static final Pose2d redSpec_RightSampleZonePos = new Pose2d(58, -45, Math.toRadians(75));
    public static final Pose2d redSpec_MidSampleZonePos = new Pose2d(58, -45, Math.toRadians(90));
    public static final Pose2d redSpec_LeftSampleZonePos = new Pose2d(50, -45, Math.toRadians(90));

    public class Lift {
        private DcMotorEx lift;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "liftMotor");
            lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(0.8);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 3000.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-0.8);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 100.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }

    public class Gripper {
        private Servo gripper;

        public Gripper(HardwareMap hardwareMap) {
            gripper = hardwareMap.get(Servo.class, "claw");
        }

        public class CloseGripper implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripper.setPosition(0.55);
                return false;
            }
        }
        public Action closeGripper() {
            return new CloseGripper();
        }

        public class OpenGripper implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripper.setPosition(1.0);
                return false;
            }
        }
        public Action openGripper() {
            return new OpenGripper();
        }
    }


    public enum AutoProgram {
        RedSpecimenAuto, RedbasketAuto
    }
    @Override
    public void runOpMode() {
        AutoProgram myAuto = AutoProgram.RedSpecimenAuto;//change
        Pose2d initialPose = new Pose2d(-15, -64, Math.toRadians(270)); //change
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Lift lift = new Lift(hardwareMap);

        // vision here that outputs position
        int visionOutputPosition = 1;


        TrajectoryActionBuilder RedSpecimenAuto =
                drive.actionBuilder(redSpec_StartPos)
                        .setTangent(90)
                        .splineToLinearHeading(redSpec_SubDepoPos, Math.toRadians(90))

//                        .waitSeconds(2) //Drop off Specimen at Submersible

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
                        .splineToLinearHeading(redSpec_SubDepoPos, Math.toRadians(90));

        TrajectoryActionBuilder RedBasketAuto =
                drive.actionBuilder(redBasket_StartPos)
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
                        .splineToLinearHeading(redBasket_AscentPos, Math.toRadians(0));
//        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
//                .lineToYSplineHeading(33, Math.toRadians(180))
//                .waitSeconds(2)
//                .strafeTo(new Vector2d(46, 30))
//                .waitSeconds(3);
//        Action trajectoryActionCloseOut = tab1.fresh()
//                .strafeTo(new Vector2d(48, 12))
//                .build();


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

//        action trajectoryActionChosen;
//        if (initialPose == redBasket_StartPos) {
//            trajectoryActionChosen = RedBasketAuto.run();
//        } else if (initialPose == redSpec_StartPos) {
//            trajectoryActionChosen = RedSpecimenAuto.run();}
//        } else {
//            trajectoryActionChosen = tab3.run();
//        }



        Action trajectoryActionChosen = null;
        initialPose = redBasket_StartPos;
        switch (myAuto) {
            case RedbasketAuto:
                trajectoryActionChosen = RedBasketAuto.build();
                break;
            case RedSpecimenAuto:
               // trajectoryActionChosen = RedSpecimenAuto.build();
                break;
            default:
                break;
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        lift.liftUp(),
                        lift.liftDown()
                )
        );
    }
}

