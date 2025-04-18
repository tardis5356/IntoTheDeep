package org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto;

import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.generateTrajectories;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redBasket_SpecimenStartPos;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redBasket_RightSampleToBasket;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redBasket_BasketToMidSample;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redBasket_MidSampleToBasket;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redBasket_BasketToLeftSample;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redBasket_LeftSampleToBasket;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redBasket_BasketToAscentPark;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redBasket_StartToSub;
import static org.firstinspires.ftc.teamcode.Echo.Auto.UticaAuto.UticaAutoTrajectories.redBasket_SubToRightSample;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
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
import org.firstinspires.ftc.teamcode.Echo.Commands.ExtendoToStateCommand;
import org.firstinspires.ftc.teamcode.Echo.Commands.IntakeCommands.IntakeInCommand;
import org.firstinspires.ftc.teamcode.Echo.Commands.LiftToStateCommand;
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

@Autonomous(name = "Delayed-BasketSpecimenAuto")
@Disabled
public class UticaCommandDelayedBasketSpecimenAuto extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    MultipleTelemetry telemetry2 = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    Pose2d initialPose = redBasket_SpecimenStartPos;

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
    private ExampleSubsystem exampleSubsystem;
    private ActionCommand RedBasket_StartToSub;
    private ActionCommand RedBasket_SubToRightSample;
    private ActionCommand RedBasket_RightSampleToBasket;
    private ActionCommand RedBasket_BasketToMidSample;
    private ActionCommand RedBasket_MidSampleToBasket;
    private ActionCommand RedBasket_BasketToLeftSample;
    private ActionCommand RedBasket_LeftSampleToBasket;
    private ActionCommand RedBasket_BasketToAscentPark;

    private InstantCommand OpenGripper;

    private InstantCommand CloseGripper;

    private InstantCommand WristSpecimen;

    private InstantCommand ArmSpecimen;

    private InstantCommand GripperCheck;

    private DepositToStateCommand depositToStateCommand;




    //    private ExampleSubsystem robot = ExampleSubsystem.getInstance();
    private boolean commandsScheduled = false;


    private ElapsedTime time_since_start;
    private double loop;

    private MecanumDriveSpecimen drive;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
//Removes previous Commands from scheduler
        CommandScheduler.getInstance().reset();

        drive = new MecanumDriveSpecimen(hardwareMap, redBasket_SpecimenStartPos); //
        telemetry.addData("Status", "Initialized");
// this line is needed or you get a Dashboard preview error
        generateTrajectories(new MecanumDriveSpecimen(hardwareMap, redBasket_SpecimenStartPos)); //
//

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
        Set<Subsystem> requirements = Set.of(exampleSubsystem);
        runtime.reset();

        intakeIn = new IntakeInCommand(intake);

        intakeOut = new IntakeInCommand(intake);

        extendoSpecLeft = new ExtendoToStateCommand(intake, extendo, "LeftSpec");

        extendoSpecMid = new ExtendoToStateCommand(intake, extendo, "MidSpec");

        extendoSpecRight = new ExtendoToStateCommand(intake, extendo, "RightSpec");

        OpenGripper = new InstantCommand(gripper::open);

        CloseGripper = new InstantCommand(gripper::close);

        WristSpecimen =  new InstantCommand(wrist::specimen);

        ArmSpecimen =  new InstantCommand(arm::specimen);

        GripperCheck = new InstantCommand(() -> gripper.checkColor());

        time_since_start = new ElapsedTime();

        RedBasket_StartToSub = new ActionCommand (redBasket_StartToSub,requirements);

        RedBasket_SubToRightSample = new ActionCommand (redBasket_SubToRightSample,requirements);

        RedBasket_RightSampleToBasket = new ActionCommand (redBasket_RightSampleToBasket, requirements);

        RedBasket_BasketToMidSample = new ActionCommand (redBasket_BasketToMidSample, requirements);

        RedBasket_MidSampleToBasket = new ActionCommand (redBasket_MidSampleToBasket, requirements);

        RedBasket_BasketToLeftSample = new ActionCommand(redBasket_BasketToLeftSample, requirements);

        RedBasket_LeftSampleToBasket = new ActionCommand (redBasket_LeftSampleToBasket, requirements);

        RedBasket_BasketToAscentPark = new ActionCommand (redBasket_BasketToAscentPark, requirements);


        CommandScheduler.getInstance().schedule(
                new InstantCommand(extendo::in),
                new InstantCommand(intake::transferPosition),
                new InstantCommand(()->lift.PIDEnabled= true),

                new SequentialCommandGroup(
                        new WaitCommand(13000),
//                        new ParallelActionCommand(arm, wrist, gripper, lift, extendo, intake, exampleSubsystem, "redBasket_StartToSub"),
//                        RedBasket_SubToRightSample,
//                        new ParallelActionCommand(arm, wrist, gripper, lift, extendo, intake, exampleSubsystem, "redBasket_IntakeRightSample"),
//                        new ParallelActionCommand(arm, wrist, gripper, lift, extendo, intake, exampleSubsystem, "redBasket_ScoreRightSample"),
//                        new ParallelActionCommand(arm, wrist, gripper, lift, extendo, intake, exampleSubsystem, "redBasket_IntakeMidSample"),
//                        new ParallelActionCommand(arm, wrist, gripper, lift, extendo, intake, exampleSubsystem, "redBasket_ScoreMidSample"),
//                        new ParallelActionCommand(arm, wrist, gripper, lift, extendo, intake, exampleSubsystem, "redBasket_IntakeLeftSample"),
//                        new ParallelActionCommand(arm, wrist, gripper, lift, extendo, intake, exampleSubsystem, "redBasket_ScoreLeftSample"),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        RedBasket_BasketToAscentPark),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new LiftToStateCommand(lift, 0, 25)))
                )
        );
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        CommandScheduler.getInstance().run();

//        if(intake.checkSample()== true){
//            intake.sIW.setPower(BotPositions.INTAKE_STOP);
//            intake.sIO.setPower(BotPositions.INTAKE_STOP);
//            intake.Intaking = false;
//            CommandScheduler.getInstance().cancelAll();
//            telemetry.addData("confirmed sample", "yes");
//        }


//        new Trigger(() -> intake.checkSample())
//                .whenActive(
//                        new SequentialCommandGroup(
//
//                                new InstantCommand(intake::stop)
//                        )
//                );

//        new Trigger(() -> intake.checkSample() && (AllianceColor.aColor == intake.checkColor() || intake.checkColor() == "yellow"))
//                .whenActive(
//                        new SequentialCommandGroup(
//                                new InstantCommand(intake::transferPosition),
//                                new InstantCommand(()->intake.sIW.setPower(.15)),
//                                new WaitCommand(500),
//                                new InstantCommand(intake::stop),
//                                new InstantCommand(extendo::in)
//                        )
//                );


        // Note: to access the drive position info, needed to declare a drive = mecanumDrive as private variable at top of this class
        telemetry.addData("In loop Heading", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.addData("X", drive.pose.position.x);
        telemetry.addData("Y", drive.pose.position.y);
        telemetry.addData("Sample acquired", intake.checkSample());

        telemetry.addData("Alliance Color", AllianceColor.aColor);

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