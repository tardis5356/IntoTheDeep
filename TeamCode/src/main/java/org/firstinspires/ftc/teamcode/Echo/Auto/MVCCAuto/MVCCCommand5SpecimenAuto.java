package org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto;

import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCSpecimenAutoTraj.generateTrajectories;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCSpecimenAutoTraj.redSpec_StartPos;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCSpecimenAutoTraj.redSpec_SubToObs4;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Echo.Auto.Tuning.MecanumDriveSpecimen;
import org.firstinspires.ftc.teamcode.Echo.Commands.DepositToStateCommand;
import org.firstinspires.ftc.teamcode.Echo.Commands.GripperAutoCloseCommand;
import org.firstinspires.ftc.teamcode.Echo.Commands.LiftToStateCommand;
import org.firstinspires.ftc.teamcode.Echo.Commands.ParallelActionCommand;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.AllianceColor;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Extendo;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.VIntake;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.TestBed.ActionCommand;
import org.firstinspires.ftc.teamcode.TestBed.ExampleSubsystem;

import java.util.Set;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name = "5SpecimenAuto")

public class MVCCCommand5SpecimenAuto extends OpMode {


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    MultipleTelemetry telemetry2 = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    Pose2d initialPose = redSpec_StartPos;

    // vision here that outputs position
    int visionOutputPosition = 1;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    private DcMotorEx mFL;
    private DcMotorEx mFR;
    private DcMotorEx mBL;
    private DcMotorEx mBR;
    private VIntake vintake;
    private Arm arm;
    private Gripper gripper;
    private Extendo extendo;
    private Lift lift;
    private Wrist wrist;
    private ExampleSubsystem exampleSubsystem;
    private ParallelActionCommand RedSpec_StartToSub;


    private ActionCommand RedSpec_SubToObs4Traj;
    private ParallelActionCommand RedSpec_SubToObs;
    private InstantCommand OpenGripper;
    private InstantCommand CloseGripper;
    private InstantCommand WristSpecimen;
    private InstantCommand ArmSpecimen;
    private InstantCommand GripperCheck;
    private DepositToStateCommand Wall;
    private GripperAutoCloseCommand gripperAutoCloseCommand;
    private ParallelActionCommand RedSpecEx_LeftSpecDepo;
    private ParallelActionCommand RedSpecEx_MidSpecDepo;
    private ParallelActionCommand RedSpecEx_RightSpecDepo;
    private ParallelActionCommand RedSpec_RightSpecDepoToObs;
    private ParallelActionCommand RedSpec_ObsToSub1;
    private ParallelActionCommand RedSpec_ObsToSub2;

    private ParallelActionCommand RedSpec_SubToObs2;
    private ParallelActionCommand RedSpec_ObsToSub3;
    private ParallelActionCommand RedSpec_SubToObs3;
    private ParallelActionCommand RedSpec_ObsToSub4;
//    private ParallelActionCommand RedSpec_SubToObs4;

    //    private ExampleSubsystem robot = ExampleSubsystem.getInstance();
    private boolean commandsScheduled = false;


    private ElapsedTime time_since_start;
    private double loop;

    private MecanumDriveSpecimen drive;
    static String botState;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //Removes previous Commands from scheduler
        CommandScheduler.getInstance().reset();

        drive = new MecanumDriveSpecimen(hardwareMap, redSpec_StartPos);
        telemetry.addData("Status", "Initialized");
        //add initial position
// this line is needed or you get a Dashboard preview error
        generateTrajectories(new MecanumDriveSpecimen(hardwareMap, redSpec_StartPos));



        vintake = new VIntake(hardwareMap);
        arm = new Arm(hardwareMap);
        gripper = new Gripper(hardwareMap);
        lift = new Lift(hardwareMap);
        extendo = new Extendo(hardwareMap);//20 inches
        wrist = new Wrist(hardwareMap);
        exampleSubsystem = new ExampleSubsystem(hardwareMap);

        Set<Subsystem> requirements = Set.of(exampleSubsystem);


        CommandScheduler.getInstance().registerSubsystem(vintake);//
        RedSpec_SubToObs4Traj = new ActionCommand(redSpec_SubToObs4, requirements);
        RedSpec_StartToSub = new ParallelActionCommand(arm, wrist, gripper, lift, extendo, vintake, exampleSubsystem, "redSpec_StartToSub");
        RedSpecEx_LeftSpecDepo = new ParallelActionCommand(arm, wrist, gripper, lift, extendo, vintake, exampleSubsystem, "redSpecEx_LeftSpecDepo");
        RedSpecEx_MidSpecDepo = new ParallelActionCommand(arm, wrist, gripper, lift, extendo, vintake, exampleSubsystem, "redSpecEx_MidSpecDepo");
        RedSpecEx_RightSpecDepo = new ParallelActionCommand(arm, wrist, gripper, lift, extendo, vintake, exampleSubsystem, "redSpecEx_RightSpecDepo");
        RedSpec_RightSpecDepoToObs = new ParallelActionCommand(arm, wrist, gripper, lift, extendo, vintake, exampleSubsystem, "redSpec_RightSpecDepoToObs");
        RedSpec_ObsToSub1 = new ParallelActionCommand(arm, wrist, gripper, lift, extendo, vintake, exampleSubsystem, "redSpec_ObsToSub1");
        RedSpec_SubToObs = new ParallelActionCommand(arm, wrist, gripper, lift, extendo, vintake, exampleSubsystem, "redSpec_SubToObs");
        RedSpec_ObsToSub2 = new ParallelActionCommand(arm, wrist, gripper, lift, extendo, vintake, exampleSubsystem, "redSpec_ObsToSub2");
        RedSpec_SubToObs2 = new ParallelActionCommand(arm, wrist, gripper, lift, extendo, vintake, exampleSubsystem, "redSpec_SubToObs2");
        RedSpec_ObsToSub3 = new ParallelActionCommand(arm, wrist, gripper, lift, extendo, vintake, exampleSubsystem, "redSpec_ObsToSub3");
        RedSpec_SubToObs3 = new ParallelActionCommand(arm, wrist, gripper, lift, extendo, vintake, exampleSubsystem, "redSpec_SubToObs3");
        RedSpec_ObsToSub4 = new ParallelActionCommand(arm, wrist, gripper, lift, extendo, vintake, exampleSubsystem, "redSpec_ObsToSub4");
//        RedSpec_SubToObs4 = new ParallelActionCommand(arm, wrist, gripper, lift, extendo, vintake, exampleSubsystem, "redSpec_SubToObs4");
        arm.sAL.setPosition(BotPositions.ARM_INTAKE);
        arm.sAR.setPosition(BotPositions.ARM_INTAKE);
        gripper.sG.setPosition(BotPositions.GRIPPER_CLOSED);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        if (gamepad1.a) {
            AllianceColor.aColor = "blue";
        }
        if (gamepad1.b) {
            AllianceColor.aColor = "red";
        }
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        botState = "specimen";

        Set<Subsystem> requirements = Set.of(exampleSubsystem);
        runtime.reset();

        generateTrajectories(new MecanumDriveSpecimen(hardwareMap, initialPose));


        OpenGripper = new InstantCommand(gripper::open);

        CloseGripper = new InstantCommand(gripper::close);

        WristSpecimen = new InstantCommand(wrist::specimen);

        ArmSpecimen = new InstantCommand(arm::specimenAuto);

        GripperCheck = new InstantCommand(() -> gripper.checkColor());

        gripperAutoCloseCommand = new GripperAutoCloseCommand(gripper);

        Wall = new DepositToStateCommand(arm, wrist, gripper, lift, "specimenToWall");

        time_since_start = new ElapsedTime();


        CommandScheduler.getInstance().schedule(
                new InstantCommand(extendo::in),
                new InstantCommand(vintake::transferPosition),
                new InstantCommand(() -> lift.PIDEnabled = true),




                new SequentialCommandGroup(
                        RedSpec_StartToSub,
//                        new InstantCommand(() -> new AngularVelConstraint(11)),//9.7
//                        new InstantCommand(() -> new ProfileAccelConstraint(-60, 80)),//-60,70
//                        new InstantCommand(() -> new TranslationalVelConstraint(100)),
                        RedSpecEx_LeftSpecDepo,
                        RedSpecEx_MidSpecDepo,
                        RedSpecEx_RightSpecDepo,
                        RedSpec_RightSpecDepoToObs,
//                        new InstantCommand(() -> new AngularVelConstraint(6.689)),
//                        new InstantCommand(() -> new ProfileAccelConstraint(-30, 50)),
//                        new InstantCommand(() -> new TranslationalVelConstraint(79)),
                        RedSpec_ObsToSub1,
                        RedSpec_SubToObs,
                        RedSpec_ObsToSub2,
                        RedSpec_SubToObs2,
                        RedSpec_ObsToSub3,
                        RedSpec_SubToObs3,
                        RedSpec_ObsToSub4,
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                new LiftToStateCommand(lift, BotPositions.LIFT_INTAKE, BotPositions.LIFT_TOLERANCE)
                                        ),
                        RedSpec_SubToObs4Traj)
                )
        );
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        CommandScheduler.getInstance().run();


        // Note: to access the drive position info, needed to declare a drive = mecanumDrive as private variable at top of this class
        telemetry.addData("In loop Heading", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.addData("X", drive.pose.position.x);
        telemetry.addData("Y", drive.pose.position.y);
        telemetry.addData("Gripper", gripper.verifyGripper());
        telemetry.addData("BotState", botState);
        telemetry.addData("Lift Position", lift.getCurrentPosition());
        telemetry.addData("Lift Target Position", lift.getTargetPosition());
        telemetry.addData("Lift localized", lift.localized);

        drive.updatePoseEstimate();
        telemetry.update();
        // Setup a variable for each drive wheel to save power level for telemetry


        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.


        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}