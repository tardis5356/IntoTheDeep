package org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto;

import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.generateTrajectories;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_LeftSpecToObs;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_MidPointToLeftSpec;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_MidSpecToObs;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_ObsToMidSpec;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_ObsToRightSpec;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_ObsToSub;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_RightSpecObsPickUpToSub;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_RightSpecToObs;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_SpecDepoToObs;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_StartPos;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_StartToSub;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_SubToMidPoint;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redSpec_SubToObs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Echo.Auto.Tuning.MecanumDriveSpecimen;
import org.firstinspires.ftc.teamcode.Echo.Commands.DepositToStateCommand;
import org.firstinspires.ftc.teamcode.Echo.Commands.GripperAutoCloseCommand;
import org.firstinspires.ftc.teamcode.Echo.Commands.ParallelActionCommand;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.AllianceColor;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Extendo;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Intake;
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

@Autonomous(name = "Delayed-SpecimenAuto")
@Disabled
public class UticaCommandDelayedSpecimenExAuto extends OpMode {


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
    private Intake intake;
    private Arm arm;
    private Gripper gripper;
    private Extendo extendo;
    private Lift lift;
    private Wrist wrist;
    private ExampleSubsystem exampleSubsystem;
    private ActionCommand RedSpec_StartToSub;
    private ActionCommand RedSpec_SubToMidPoint;
    private ActionCommand RedSpec_MidPointToLeftSpec;
    private ActionCommand RedSpec_RightSpecToObs;
    private ActionCommand RedSpec_SpecDepoToObs;
    private ActionCommand RedSpec_ObsToRightSpec;
    private ActionCommand RedSpec_RightSpecObsPickUpToSub;
    private ActionCommand RedSpec_LeftSpecToObs;
    private ActionCommand RedSpec_MidSpecToObs;
    private ActionCommand RedSpec_ObsToMidSpec;
    private ActionCommand RedSpec_ObsToSub;
    private ActionCommand RedSpec_SubToObs;

    private InstantCommand OpenGripper;

    private InstantCommand CloseGripper;

    private InstantCommand WristSpecimen;

    private InstantCommand ArmSpecimen;

    private InstantCommand GripperCheck;

    private DepositToStateCommand Wall;

    private GripperAutoCloseCommand gripperAutoCloseCommand;


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


        intake = new Intake(hardwareMap);
        arm = new Arm(hardwareMap);
        gripper = new Gripper(hardwareMap);
        lift = new Lift(hardwareMap);
        extendo = new Extendo(hardwareMap);//20 inches
        wrist = new Wrist(hardwareMap);
        exampleSubsystem = new ExampleSubsystem(hardwareMap);

        Set<Subsystem> requirements = Set.of(exampleSubsystem);


        CommandScheduler.getInstance().registerSubsystem(intake);//
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

        RedSpec_StartToSub = new ActionCommand(redSpec_StartToSub, requirements);

        RedSpec_SubToMidPoint = new ActionCommand(redSpec_SubToMidPoint, requirements);

        RedSpec_LeftSpecToObs = new ActionCommand(redSpec_LeftSpecToObs, requirements);

        RedSpec_MidPointToLeftSpec = new ActionCommand(redSpec_MidPointToLeftSpec, requirements);

        RedSpec_MidSpecToObs = new ActionCommand(redSpec_MidSpecToObs, requirements);

        RedSpec_ObsToMidSpec = new ActionCommand(redSpec_ObsToMidSpec, requirements);

        RedSpec_ObsToRightSpec = new ActionCommand(redSpec_ObsToRightSpec, requirements);

        RedSpec_RightSpecObsPickUpToSub = new ActionCommand(redSpec_RightSpecObsPickUpToSub, requirements);

        RedSpec_SpecDepoToObs = new ActionCommand(redSpec_SpecDepoToObs, requirements);

        RedSpec_RightSpecToObs = new ActionCommand(redSpec_RightSpecToObs, requirements);

        RedSpec_ObsToSub = new ActionCommand(redSpec_ObsToSub, requirements);

        RedSpec_SubToObs = new ActionCommand(redSpec_SubToObs, requirements);

        OpenGripper = new InstantCommand(gripper::open);

        CloseGripper = new InstantCommand(gripper::close);

        WristSpecimen = new InstantCommand(wrist::specimen);

        ArmSpecimen = new InstantCommand(arm::specimen);

        GripperCheck = new InstantCommand(() -> gripper.checkColor());

        gripperAutoCloseCommand = new GripperAutoCloseCommand(gripper);

        Wall = new DepositToStateCommand(arm, wrist, gripper, lift, "specimenToWall");

        time_since_start = new ElapsedTime();


        CommandScheduler.getInstance().schedule(
                new InstantCommand(extendo::in),
                new InstantCommand(intake::transferPosition),
                new InstantCommand(() -> lift.PIDEnabled = true),

                new SequentialCommandGroup(
                        new WaitCommand(13000),
                        new ParallelActionCommand(arm, wrist, gripper, lift, extendo, intake, exampleSubsystem, "redSpec_StartToSub"),
                        new ParallelActionCommand(arm, wrist, gripper, lift, extendo, intake, exampleSubsystem, "redSpecEx_LeftSpecDepo"),
                        new ParallelActionCommand(arm, wrist, gripper, lift, extendo, intake, exampleSubsystem, "redSpecEx_MidSpecDepo"),
                        new ParallelActionCommand(arm, wrist, gripper, lift, extendo, intake, exampleSubsystem, "redSpecEx_RightSpecDepo"),
                        new ParallelActionCommand(arm, wrist, gripper, lift, extendo, intake, exampleSubsystem, "redSpec_RightSpecDepoToObs"),
                        new ParallelActionCommand(arm, wrist, gripper, lift, extendo, intake, exampleSubsystem, "redSpec_ObsToSub1"),
                        new ParallelActionCommand(arm, wrist, gripper, lift, extendo, intake, exampleSubsystem, "redSpec_SubToObs"),
                        new ParallelActionCommand(arm, wrist, gripper, lift, extendo, intake, exampleSubsystem, "redSpec_ObsToSub2"),
                        new ParallelActionCommand(arm, wrist, gripper, lift, extendo, intake, exampleSubsystem, "redSpec_SubToObs2"),
                        new ParallelActionCommand(arm, wrist, gripper, lift, extendo, intake, exampleSubsystem, "redSpec_ObsToSub3")
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
        telemetry.addData("Lift Position", lift.getTargetPosition());
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