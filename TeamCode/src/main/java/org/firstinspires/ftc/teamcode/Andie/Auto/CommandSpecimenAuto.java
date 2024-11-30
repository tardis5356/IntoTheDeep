package org.firstinspires.ftc.teamcode.Andie.Auto;

import static org.firstinspires.ftc.teamcode.TestBed.AutoPathing.AutoTrajectories.generateTrajectories;
import static org.firstinspires.ftc.teamcode.TestBed.AutoPathing.AutoTrajectories.redSpec_LeftMidSpecToObs;
import static org.firstinspires.ftc.teamcode.TestBed.AutoPathing.AutoTrajectories.redSpec_LeftMidSpecToObs;
import static org.firstinspires.ftc.teamcode.TestBed.AutoPathing.AutoTrajectories.redSpec_ObsToRightSpec;
import static org.firstinspires.ftc.teamcode.TestBed.AutoPathing.AutoTrajectories.redSpec_ObsToSub;
import static org.firstinspires.ftc.teamcode.TestBed.AutoPathing.AutoTrajectories.redSpec_Park;
import static org.firstinspires.ftc.teamcode.TestBed.AutoPathing.AutoTrajectories.redSpec_StartPos;
import static org.firstinspires.ftc.teamcode.TestBed.AutoPathing.AutoTrajectories.redSpec_StartToSub;
import static org.firstinspires.ftc.teamcode.TestBed.AutoPathing.AutoTrajectories.redSpec_SubToLeftMidSpec;
import static org.firstinspires.ftc.teamcode.TestBed.AutoPathing.AutoTrajectories.redSpec_SubToObs;
import static org.firstinspires.ftc.teamcode.TestBed.AutoPathing.AutoTrajectories.redSpec_RightSpecToObs;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Andie.Commands.DepositToStateCommand;
import org.firstinspires.ftc.teamcode.Andie.Commands.ExtendoToStateCommand;
import org.firstinspires.ftc.teamcode.Andie.Commands.IntakeInCommand;
import org.firstinspires.ftc.teamcode.TestBed.ActionCommand;
//import org.firstinspires.ftc.teamcode.TestBed.AutoPathing.RedSpecimenAuto;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Extendo;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.TestBed.ExampleSubsystem;
import org.firstinspires.ftc.teamcode.TestBed.Tuning.MecanumDrive;

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

@Autonomous(name = "SpecimenAuto")

public class CommandSpecimenAuto extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    MultipleTelemetry telemetry2 = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    Pose2d initialPose = redSpec_StartPos;

    // vision here that outputs position
    int visionOutputPosition = 1;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    ExtendoToStateCommand extendoToStateCommand;

    ExtendoToStateCommand extendoSpecLeft;
    ExtendoToStateCommand extendoSpecMid;
    ExtendoToStateCommand extendoSpecRight;

    IntakeInCommand intakeIn;
    IntakeInCommand intakeOut;
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
    private Subsystem ExampleSubsystem;
    private ActionCommand RedSpec_StartToSub;
    private ActionCommand RedSpec_SubToLeftMidSpec;
    private ActionCommand RedSpec_RightSpecToObs;
    private ActionCommand RedSpec_ObsToRightSpec;
    private ActionCommand RedSpec_LeftMidSpecToObs;
    private ActionCommand RedSpec_ObsToSub;
    private ActionCommand RedSpec_SubToObs;
    private ActionCommand RedSpec_Park;

    private DepositToStateCommand depositToStateCommand;


    //    private ExampleSubsystem robot = ExampleSubsystem.getInstance();
    private boolean commandsScheduled = false;


    private ElapsedTime time_since_start;
    private double loop;

    private MecanumDrive drive;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, redSpec_StartPos);
        telemetry.addData("Status", "Initialized");
// this line is needed or you get a Dashboard preview error
        generateTrajectories(new MecanumDrive(hardwareMap, redSpec_StartPos));


        intake = new Intake(hardwareMap);
        arm = new Arm(hardwareMap);
        gripper = new Gripper(hardwareMap);
        lift = new Lift(hardwareMap);
        extendo = new Extendo(hardwareMap);//20 inches
        wrist = new Wrist(hardwareMap);
        ExampleSubsystem = new ExampleSubsystem(hardwareMap);

        Set<Subsystem> requirements = Set.of(ExampleSubsystem);


        CommandScheduler.getInstance().registerSubsystem(intake);//
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {


        intakeIn = new IntakeInCommand(intake);

        intakeOut = new IntakeInCommand(intake);

        extendoSpecLeft = new ExtendoToStateCommand(intake, extendo, "LeftSpec");

        extendoSpecMid = new ExtendoToStateCommand(intake, extendo, "MidSpec");

        extendoSpecRight = new ExtendoToStateCommand(intake, extendo, "RightSpec");

        Set<Subsystem> requirements = Set.of(ExampleSubsystem);
        runtime.reset();

        generateTrajectories(new MecanumDrive(hardwareMap, initialPose));

        RedSpec_StartToSub = new ActionCommand(redSpec_StartToSub, requirements);//

        RedSpec_SubToLeftMidSpec = new ActionCommand(redSpec_SubToLeftMidSpec, requirements);//

        RedSpec_LeftMidSpecToObs = new ActionCommand(redSpec_LeftMidSpecToObs, requirements);//

        RedSpec_ObsToRightSpec = new ActionCommand(redSpec_ObsToRightSpec, requirements);

        RedSpec_RightSpecToObs = new ActionCommand(redSpec_RightSpecToObs, requirements);//

        RedSpec_ObsToSub = new ActionCommand(redSpec_ObsToSub, requirements);//

        RedSpec_SubToObs = new ActionCommand(redSpec_SubToObs, requirements);//

        RedSpec_Park = new ActionCommand(redSpec_Park, requirements);//

        time_since_start = new ElapsedTime();


        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(//REMEMBER TO ADD
                         RedSpec_StartToSub,// arm and wrist and gripper all go to intake position
                        //hang the specimen
                        RedSpec_SubToLeftMidSpec,
                        RedSpec_LeftMidSpecToObs,
//                        intakeOut,
//                        //Pick up sample
//                        extendoSpecMid,
//                        intakeIn,
//                        //drop off sample in Obs Zone
//                        intakeOut,
//                        //Pick up sample
                        RedSpec_ObsToRightSpec,
//                        extendoSpecRight,
//                        intakeIn,
////                // drop off sample in Obs Zone
                        RedSpec_RightSpecToObs,
//                        intakeOut,
////                //pick and drop right spec
                        RedSpec_ObsToSub,
        ////                //hang the specimen
                        RedSpec_SubToObs,
        ////                //pick second specimen
                        RedSpec_ObsToSub,
        ////                //hang the specimen
                        RedSpec_SubToObs,
        ////                //pick second specimen
                        RedSpec_ObsToSub,
                        RedSpec_SubToObs
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