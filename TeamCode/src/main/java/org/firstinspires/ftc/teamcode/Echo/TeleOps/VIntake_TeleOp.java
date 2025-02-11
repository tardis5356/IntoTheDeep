package org.firstinspires.ftc.teamcode.Echo.TeleOps;

import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCSpecimenAutoTraj.generateTrajectories;
import static org.firstinspires.ftc.teamcode.Echo.Auto.MVCCAuto.MVCCSpecimenAutoTraj.redSpec_StartPos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Echo.Auto.Tuning.MecanumDriveSpecimen;
import org.firstinspires.ftc.teamcode.Echo.Commands.DepositToStateCommand;
import org.firstinspires.ftc.teamcode.Echo.Commands.IntakeCommands.IntakeOutCommand;
import org.firstinspires.ftc.teamcode.Echo.Commands.IntakeCommands.IntakePassCommand;
import org.firstinspires.ftc.teamcode.Echo.Commands.IntakeToStateCommand;
import org.firstinspires.ftc.teamcode.Echo.Commands.LiftToStateCommand;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.AllianceColor;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Extendo;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.VIntake;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Winch;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Wrist;
import org.firstinspires.ftc.teamcode.TestBed.ExampleSubsystem;

import java.util.Set;

@Config
@TeleOp(name = "BP_TeleOp", group = "AGen1")

public class VIntake_TeleOp extends CommandOpMode {
    //gamepads
    //GamepadEx is an extended object version of gamepads that has more organized input checks that we use in triggers.
    private GamepadEx driver1, driver2;
    //private DcMotorEx liftEncoder;

    //This is just a boolean used for telemetry to see if we took in the incorrect sample color
    boolean wrongColorIntaked = false;

    boolean killSwitchPressed = false;


    boolean killSwitchActive = false;

    boolean outaking = false;

    String notAColor;

    boolean gOpen;

    //private IntakeInCommand intakeInCommand;

    //drivetrain motors and variables
    //DcMotorEx is an expanded version of the DcMotor variable that gives us more methods.
    //For example, stop and reset encoder.
    private DcMotorEx mFL, mFR, mBL, mBR;

    private Servo sEL, sER, sIG;

    //Forward and back power, Left and right power, rotation power.
    //All are then added and subtracted in different ways for each drive motor
    double FB, LR, Rotation;

    //multipliers applied to the sum of the above variables to evenly change the speed of the drivetrain
    static double FAST_SPEED_MULTIPLIER = 1;
    static double SLOW_SPEED_MULTIPLIER = 0.4;

    //CURRENT_SPEED_MULTIPLIER is the actual multiplier applied to the drive train power. It is set to either the fast or slow multipliers
    double CURRENT_SPEED_MULTIPLIER;


    //below we create a new object instance of all the subsystem classes
    //gripper
    private Gripper gripper;

    //lift
    private Lift lift;

    //wrist
    private Wrist wrist;

    //intake
    private VIntake vintake;

    private Extendo extendo;

    private Arm arm;
    private Winch winch;

    private MecanumDriveSpecimen drive;

    //private ColorSensor cI;

    //private TouchSensor limitLift;

    //public Boolean TeamColorRed;

    //This is the sum of the two driver1 trigger values that is then added to the position of the extension servos
    double Trigger;

    //stores the current configuration of the deposit half of the robot.
    //This is then used to determine which DepositToStateCommand should be called.
    static String DepositState;

    boolean IntakeToggle = true;


    //Commands are also objects, and thus new instances need to be made for new files
    public DepositToStateCommand intakeToIntake, wallToIntake, basketToIntake, specimenToIntake;
    public DepositToStateCommand intakeToWallWithSomething, intakeToWallWithNothing, basketToWall, specimenToWall;
    public DepositToStateCommand intakeToSpecimen, basketToSpecimen, wallToSpecimen;
    public DepositToStateCommand intakeToBasketLow, wallToBasketLow, specimenToBasketLow, basketHighToBasketLow;
    public DepositToStateCommand intakeToBasketHigh, wallToBasketHigh, specimenToBasketHigh, basketLowToBasketHigh;


    public IntakeOutCommand intakeOutCommand;

    //stores the values of the left and right triggers for calculating the trigger value for the extendo
    double LeftTrigger;
    double RightTrigger;

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    MultipleTelemetry telemetry2 = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    int visionOutputPosition = 1;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    private ExampleSubsystem exampleSubsystem;
    @Override
    //stuff that is ran when you click init at the start of teleop.
    public void initialize(){

        //Removes previous Commands from scheduler
        //We call it at the start of TeleOp as it clears lingering autocommands that make the intake freak out
        //Make sure it is not called in a loop since it will clear all the triggers every frame. Be very careful. It is a kill switch.
        CommandScheduler.getInstance().reset();

        //sets the digital position of the robot to intake for the deposit to state command
        DepositState = "intake";

        //init controllers
        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        //call the hardware map for all the subsystems
        extendo = new Extendo(hardwareMap);

        //gripper
        gripper = new Gripper(hardwareMap);

        //lift
        lift = new Lift(hardwareMap);

        //wrist
        wrist = new Wrist(hardwareMap);

        //intake
        vintake = new VIntake(hardwareMap);

        //intake
        arm = new Arm(hardwareMap);
        //drive = new MecanumDriveSpecimen(hardwareMap, redSpec_StartPos);
        winch = new Winch(hardwareMap);
//        generateTrajectories(new MecanumDriveSpecimen(hardwareMap, redSpec_StartPos));
//        exampleSubsystem = new ExampleSubsystem(hardwareMap);
//
//        Set<Subsystem> requirements = Set.of(exampleSubsystem);
        //TeamColorRed = true;

        //intakeInCommand = new IntakeInCommand(intake);
        //also hardware map the depositToStateCommand. Although we do create new instances in the triggers so I don't think that's needed
        intakeToIntake = new DepositToStateCommand(arm,wrist, gripper, lift, "intakeToIntake");
        wallToIntake = new DepositToStateCommand(arm,wrist,gripper,lift, "wallToIntake");
        basketToIntake = new DepositToStateCommand(arm,wrist,gripper,lift, "basketToIntake");
        specimenToIntake = new DepositToStateCommand(arm,wrist,gripper,lift, "specimenToIntake");

        specimenToWall = new DepositToStateCommand(arm,wrist,gripper,lift,"specimenToWall");
        basketToWall = new DepositToStateCommand(arm, wrist, gripper,lift, "basketToWall");
        intakeToWallWithNothing = new DepositToStateCommand(arm, wrist, gripper,lift, "intakeToWallWithNothing");
        intakeToWallWithSomething = new DepositToStateCommand(arm, wrist, gripper,lift, "intakeToWallWithSomething");

        wallToSpecimen = new DepositToStateCommand(arm,wrist,gripper,lift,"wallToSpecimen");
        intakeToSpecimen = new DepositToStateCommand(arm,wrist,gripper,lift,"intakeToSpecimen");
        basketToSpecimen = new DepositToStateCommand(arm,wrist,gripper,lift,"basketToSpecimen");

        intakeToBasketLow = new DepositToStateCommand(arm,wrist,gripper,lift,"intakeToBasketLow");
        wallToBasketLow = new DepositToStateCommand(arm,wrist,gripper,lift,"wallToBasketLow");
        specimenToBasketLow = new DepositToStateCommand(arm,wrist,gripper,lift,"specimenToBasketLow");
        basketHighToBasketLow = new DepositToStateCommand(arm,wrist,gripper,lift,"basketHighToBasketLow");

        intakeToBasketHigh = new DepositToStateCommand(arm,wrist,gripper,lift,"intakeToBasketHigh");
        wallToBasketHigh = new DepositToStateCommand(arm,wrist,gripper,lift,"wallToBasketHigh");
        specimenToBasketHigh = new DepositToStateCommand(arm,wrist,gripper,lift,"specimenToBasketHigh");
        basketLowToBasketHigh = new DepositToStateCommand(arm,wrist,gripper,lift,"basketLowToBasketHigh");





        //map motors
        mFL = hardwareMap.get(DcMotorEx.class, "mFL");
        mFR = hardwareMap.get(DcMotorEx.class, "mFR");
        mBL = hardwareMap.get(DcMotorEx.class, "mBL");
        mBR = hardwareMap.get(DcMotorEx.class, "mBR");

        //liftEncoder = hardwareMap.get(DcMotorEx.class, "");
        //cI = hardwareMap.get(ColorSensor.class, "cI");
        //limitLift = hardwareMap.get(TouchSensor.class, "lL");
        //this motor physically runs opposite. For convenience, reverse direction.
        mBR.setDirection(DcMotorSimple.Direction.REVERSE);
        mFR.setDirection(DcMotorSimple.Direction.REVERSE);

        //makes the motors brake when power = zero. Is better for driver precision
        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        sER = hardwareMap.get(Servo.class, "sER");
        sEL = hardwareMap.get(Servo.class, "sEL");
        //sIG = hardwareMap.get(Servo.class, "sIG");



        //new DepositToStateCommand(arm,wrist,gripper,lift,"transit");

        //This will drive the arm, wrist, and gripper, to the intake position for the start of teleop
        //arm.hang();
        wrist.intake();
        gripper.intake();
        vintake.transferPosition();

        gOpen = true;

        //Changes if the drivetrain is in fast mode or slow mode. Thx Graham!
        CURRENT_SPEED_MULTIPLIER = FAST_SPEED_MULTIPLIER;

        new Trigger(() -> driver1.getButton(GamepadKeys.Button.B))
                .toggleWhenActive(() -> CURRENT_SPEED_MULTIPLIER = FAST_SPEED_MULTIPLIER, ()-> CURRENT_SPEED_MULTIPLIER = SLOW_SPEED_MULTIPLIER);

    //global variables
        {
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.DPAD_LEFT))
                .whenActive(
                        new InstantCommand(()->AllianceColor.aColor = "blue")
                );

        new Trigger(() -> driver1.getButton(GamepadKeys.Button.DPAD_RIGHT))
                .whenActive(
                        new InstantCommand(()->AllianceColor.aColor = "red")
                );

        new Trigger(() -> driver1.getButton(GamepadKeys.Button.DPAD_UP))
                .whenActive(new InstantCommand(()-> AllianceColor.cycleType = "basket"));

        new Trigger(() -> driver1.getButton(GamepadKeys.Button.DPAD_DOWN))
                .whenActive(new InstantCommand(()-> AllianceColor.cycleType = "specimen"));}

        //gripper Commands
        {
            new Trigger(()-> gripper.verifyGripper())
                    .whenActive(new InstantCommand(()-> gOpen = false));




            //if driver 2 presses b, toggle between open and closed
            new Trigger(() -> driver2.getButton(GamepadKeys.Button.B) && DepositState != "intake" && !gOpen)
                    .whenActive(new SequentialCommandGroup(
                            new InstantCommand(gripper::open),
                            new WaitCommand(300),
                            new InstantCommand(()->gOpen = true)
                    ));

            new Trigger(() -> driver2.getButton(GamepadKeys.Button.B) && gOpen )
                    .whenActive(new SequentialCommandGroup(
                            new InstantCommand(gripper::close),
                            new WaitCommand(300),
                            new InstantCommand(()->gOpen = false)
                    ));

            new Trigger(() -> driver2.getButton(GamepadKeys.Button.B) && DepositState == "intake" && !gOpen)
                    .whenActive(new SequentialCommandGroup(
                            new InstantCommand(gripper::intake),
                            new WaitCommand(300),
                            new InstantCommand(()->gOpen = true)
                    ));


            new Trigger(() -> driver2.getButton(GamepadKeys.Button.Y) && DepositState == "specimen" && !gOpen)
                    .whenActive(
                            new SequentialCommandGroup(
                                    new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH_CLIP, 20),
                                    new InstantCommand(gripper::open)
                            )
                    );

         //   new Trigger(() -> (gripper.verifyJig() && DepositState == "intake") || (DepositState == "wall" && gripper.verifyGripper()))
         //           .whenActive(new InstantCommand(gripper::close));
        }

        //intake
        {
//            new Trigger (() -> AllianceColor.aColor == "blue")
//                    .whenActive(new InstantCommand(intake::checkBlue));

            //intake tilting
            //if the extendo is outside the robot and the driver is trying to tilt the intake, toggle between up and down
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.LEFT_BUMPER) && extendo.sER.getPosition()<=.72 && vintake.wasRaised)
                .whenActive(
                        //we have sequences for the tilting to make sure that the wrist of the intake moves first before the arm
                        //that's done so we don't the intake pinned against the ground
                            new SequentialCommandGroup(
                                new InstantCommand(vintake::downPosition))

                );
        new Trigger(()-> driver1.getButton(GamepadKeys.Button.LEFT_BUMPER) && extendo.sER.getPosition()<=.72 && !vintake.wasRaised)
                .whenActive(
                        new SequentialCommandGroup(
                                new InstantCommand(vintake::upPosition)
                        )
                );

        //This trigger is if the extension is close to the robot, the intake automatically goes to be in the transfer position
        new Trigger(() -> extendo.sER.getPosition() >= .62)
                .whileActiveOnce(
                        new SequentialCommandGroup(
                        new InstantCommand(vintake::upPosition)
                        //new WaitCommand(200),
                        //new InstantCommand(extendo::in),
                        //new WaitCommand(300),
                        /*new InstantCommand(intake::transfer)*/)
                );




        //intake inning and outing


            //if d1 hits right bumper, there are no commands to outake, and the toggle is true
        new Trigger(() -> (driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER)) && (!driver2.getButton(GamepadKeys.Button.LEFT_BUMPER) || !driver1.getButton(GamepadKeys.Button.Y))&& IntakeToggle == true)
                .whenActive(new SequentialCommandGroup(new InstantCommand(vintake::in),
                        new WaitCommand(30),
                        new InstantCommand(() -> IntakeToggle = false)
                ));

        //if d1 hits right bumper, there are no commands to outake, and the toggle is false
            new Trigger(() -> (driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER))&& (!driver2.getButton(GamepadKeys.Button.LEFT_BUMPER) || !driver1.getButton(GamepadKeys.Button.Y))&& IntakeToggle == false)
                    .whenActive( new SequentialCommandGroup(new InstantCommand(vintake::stop),
                            new WaitCommand(30),
                            new InstantCommand(() -> IntakeToggle = true)
                    ));



            //TODO: Change this one if we do the pass through

            //if either the right bumpers are down AND there isn't a detected sample AND neither driver2's left bumper or driver1's y button are down
            //toggle between running the intake and not
            new Trigger(() -> driver2.getButton(GamepadKeys.Button.LEFT_BUMPER) || driver1.getButton(GamepadKeys.Button.Y) || ((vintake.checkColor() == "red" && AllianceColor.aColor == "blue") || (vintake.checkColor() == "blue" && AllianceColor.aColor == "red")))
                    .whenActive(
                            new SequentialCommandGroup(
                                    //new InstantCommand(()-> intake.samplePresent = false),
                               //     new InstantCommand(()-> outaking = true),
                                    new InstantCommand(vintake::upPosition),
                                    new InstantCommand(vintake::out),
                                    new InstantCommand(() -> IntakeToggle = false)
                      //              new WaitCommand(2000),
                                  //  new InstantCommand(()-> outaking = false)
                                    //new IntakeOutCommand(intake)
                            )
                    );}

        //if the intake detects a sample and we haven't disabled the samplePresent variable for transferring
        //stop intaking and outake just the spokes to make sure we don't accidentally nab 2 samples


        //transfer
        {
            //new Trigger(()-> intake.checkSample() && DepositState == "intake")
            //       .whenActive(new InstantCommand(intake::transfer));

        //If driver1 hits a or driver2 hits x, run the intake pass or transfer command
        new Trigger(()-> driver1.getButton(GamepadKeys.Button.A) || driver2.getButton(GamepadKeys.Button.RIGHT_BUMPER))
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(vintake::transferPosition),
                        new InstantCommand(vintake::in),
                        new WaitCommand(1000),
                        new InstantCommand(vintake::stop)
                        ));

            //if the gripper detects a sample while the depositing system is in the intake configuration, the intake should
            //transfer the sample.
//            new Trigger(()-> gripper.verifyGripper() && DepositState == "intake")
//                    .whenActive(
//                            new SequentialCommandGroup(
//                                    new WaitCommand(250),
//                                    new InstantCommand(intake::transfer)
//                            )
//                    );

        }

        //Extendo
        {
        //if driver1 presses in on the right stick, toggle between the extendo being all the way out or all the way in.
        new Trigger(() -> driver1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON) && extendo.extensionPosition >= .78)
                .whenActive(
                        //this sequential command group is here to make sure the intake doesn't hit the drivetrain on the way in
                        //and to make sure any samples don't get spat out.
                        new SequentialCommandGroup(

                                new InstantCommand(vintake::in),
                                new InstantCommand(extendo::out),
                                new InstantCommand(vintake::upPosition),
                                new InstantCommand(()-> IntakeToggle = true)
                        )
                );

        new Trigger(()->driver1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON) && extendo.extensionPosition < .78)
                .whenActive(new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(vintake::stop),
                                new InstantCommand(vintake::upPosition)
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(100),
                                new InstantCommand(extendo::in)
                        )
                ));

        //if the intake detects a sample and its the right alliance color, or is yellow, automatically drive the extendo into the robot
        //and have the sample be slightly popped out. This is really cool actually as the extendo brings the sample right into the gripper.
        new Trigger(() -> AllianceColor.cycleType == "basket" && vintake.checkSample() && ((vintake.checkColor() == "red" && AllianceColor.aColor == "red") || (vintake.checkColor() == "blue" && AllianceColor.aColor == "blue") || vintake.checkColor() == "yellow"))
                .whenActive(
                        new SequentialCommandGroup(
                                        new InstantCommand(vintake::out),
                                        new InstantCommand(vintake::upPosition),
                                        new WaitCommand(250),
                                        new InstantCommand(vintake::in),
                                        new InstantCommand(extendo::in),
                                        new WaitCommand(500),
                                        new InstantCommand(vintake::stop)


                ));
            new Trigger(() -> AllianceColor.cycleType == "specimen" && vintake.checkSample() && ((vintake.checkColor() == "red" && AllianceColor.aColor == "red") || (vintake.checkColor() == "blue" && AllianceColor.aColor == "blue")))
                    .whenActive(
                            new SequentialCommandGroup(

                                            new InstantCommand(vintake::out),
                                            new InstantCommand(vintake::upPosition),
                                            new WaitCommand(250),
                                            new InstantCommand(vintake::in),
                                            new InstantCommand(extendo::in),
                                            new WaitCommand(500),
                                            new InstantCommand(vintake::stop)


                            ));
        }
//
//        //Deposit to state commands
        {
        //each starting position to end position command needs its own trigger.
        //all the triggers check the driver input and the current state of the robot to tell if they should be called.

        //ToIntakeCommands
        new Trigger(() -> driver2.getButton(GamepadKeys.Button.A) && (DepositState == "basketHigh" || DepositState == "basketLow"))
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(()->lift.PIDEnabled= true),
                        basketToIntake,
                        new InstantCommand(() -> DepositState = "intake"),
                        new InstantCommand(() -> gOpen = true),
                        new InstantCommand(() -> killSwitchActive = false)
                ));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.A) && DepositState == "wall")
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(()->lift.PIDEnabled= true),
                        wallToIntake,
                        new InstantCommand(() -> DepositState = "intake"),
                        new InstantCommand(() -> gOpen = true),
                        new InstantCommand(() -> killSwitchActive = false)
                ));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.A) && DepositState == "specimen")
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(()->lift.PIDEnabled= true),
                        specimenToIntake,
                        new InstantCommand(() -> DepositState = "intake"),
                        new InstantCommand(() -> gOpen = true),
                        new InstantCommand(() -> killSwitchActive = false)
                ));
                //.cancelWhenActive(specimenToIntake);

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.A) && DepositState == "intake")
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(()->lift.PIDEnabled= true),
                        intakeToIntake,
                        new InstantCommand(() -> DepositState = "intake"),
                        new InstantCommand(() -> gOpen = true),
                        new InstantCommand(() -> killSwitchActive = false)
                ));

        //ToWallCommands
        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_LEFT) && (DepositState == "basketHigh" || DepositState == "basketLow"))
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(()->lift.PIDEnabled= true),
                        basketToWall,
                        new InstantCommand(() -> DepositState = "wall"),
                        new InstantCommand(() -> gOpen = true),
                        new InstantCommand(() -> killSwitchActive = false)
                ));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_LEFT) && DepositState == "intake" && gripper.verifyGripper())
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(()->lift.PIDEnabled= true),
                        new InstantCommand(vintake::in),
                        intakeToWallWithSomething,
                        new InstantCommand(vintake::stop),
                        new InstantCommand(() -> DepositState = "wall"),
                        new InstantCommand(() -> gOpen = false),
                        new InstantCommand(() -> killSwitchActive = false)
                ));

            new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_LEFT) && DepositState == "intake" && !gripper.verifyGripper())
                    .whenActive(new SequentialCommandGroup(
                            new InstantCommand(()->lift.PIDEnabled= true),
                            intakeToWallWithNothing,
                            new InstantCommand(() -> DepositState = "wall"),
                            new InstantCommand(() -> gOpen = true),
                            new InstantCommand(() -> killSwitchActive = false)
                    ));


        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_LEFT) && DepositState == "specimen")
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(()->lift.PIDEnabled= true),
                        specimenToWall,
                        new InstantCommand(() -> DepositState = "wall"),
                        new InstantCommand(() -> gOpen = true),
                        new InstantCommand(() -> killSwitchActive = false)
                ));

        //ToHighBasket Commands
        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_UP) && DepositState == "wall")
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(()->lift.PIDEnabled= true),
                        wallToBasketHigh,
                        //new DepositToStateCommand(arm, wrist, gripper, lift,"wallToBasketHigh"),
                        new InstantCommand(() -> DepositState = "basketHigh"),
                        new InstantCommand(() -> killSwitchActive = false)
                ));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_UP) && DepositState == "intake")
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(()->lift.PIDEnabled= true),
                        new InstantCommand(vintake::in),
                        intakeToBasketHigh,
                        //new DepositToStateCommand(arm, wrist, gripper, lift,"intakeToBasketHigh"),
                        new InstantCommand(vintake::stop),
                        new InstantCommand(() -> DepositState = "basketHigh"),
                        new InstantCommand(() -> killSwitchActive = false)
                ));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_UP) && DepositState == "specimen")
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(()->lift.PIDEnabled= true),
                        specimenToBasketHigh,
                        //new DepositToStateCommand(arm, wrist, gripper, lift,"specimenToBasketHigh"),
                        new InstantCommand(() -> DepositState = "basketHigh"),
                        new InstantCommand(() -> killSwitchActive = false)
                ));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_UP) && DepositState == "basketLow")
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(()->lift.PIDEnabled= true),
                        basketLowToBasketHigh,
                        //new DepositToStateCommand(arm, wrist, gripper, lift,"basketLowToBasketHigh"),
                        new InstantCommand(() -> DepositState = "basketHigh"),
                        new InstantCommand(() -> killSwitchActive = false)
                ));

        //To Low Basket Commands
        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_DOWN) && DepositState == "basketHigh")
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(()->lift.PIDEnabled= true),
                        basketHighToBasketLow,
                        //new DepositToStateCommand(arm, wrist, gripper, lift,"basketHighToBasketLow"),
                        new InstantCommand(() -> DepositState = "basketLow"),
                        new InstantCommand(() -> killSwitchActive = false)
                ));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_DOWN) && DepositState == "intake")
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(()->lift.PIDEnabled= true),
                        new InstantCommand(vintake::in),
                        intakeToBasketLow,
                        //new DepositToStateCommand(arm, wrist, gripper, lift,"intakeToBasketLow"),
                        new InstantCommand(vintake::stop),
                        new InstantCommand(() -> DepositState = "basketLow"),
                        new InstantCommand(() -> killSwitchActive = false)
                ));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_DOWN) && DepositState == "wall")
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(()->lift.PIDEnabled= true),
                        wallToBasketLow,
                        //new DepositToStateCommand(arm, wrist, gripper, lift,"wallToBasketLow"),
                        new InstantCommand(() -> DepositState = "basketLow"),
                        new InstantCommand(() -> killSwitchActive = false)
                ));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_DOWN) && DepositState == "specimen")
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(()->lift.PIDEnabled= true),
                        specimenToBasketLow,
                        //new DepositToStateCommand(arm, wrist, gripper, lift,"specimenToBasketLow"),
                        new InstantCommand(() -> DepositState = "basketLow"),
                        new InstantCommand(() -> killSwitchActive = false)
                ));

        //To high specimen commands

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_RIGHT) && (DepositState == "basketHigh" || DepositState == "basketLow"))
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(()->lift.PIDEnabled= true),
                        basketToSpecimen,
                        //new DepositToStateCommand(arm, wrist, gripper, lift,"basketToSpecimen"),
                        new InstantCommand(() -> DepositState = "specimen"),
                        new InstantCommand(() -> killSwitchActive = false)
                ));

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_RIGHT) && DepositState == "wall")
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(()->lift.PIDEnabled= true),
                        wallToSpecimen,
                        //new DepositToStateCommand(arm, wrist, gripper, lift,"wallToSpecimen"),
                        new InstantCommand(() -> DepositState = "specimen"),
                        new InstantCommand(() -> killSwitchActive = false)
                ));

            new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_RIGHT) && DepositState == "specimen")
                    .whenActive(new SequentialCommandGroup(
                            new InstantCommand(()->lift.PIDEnabled= true),
                           new InstantCommand(arm::specimenHang),
                            new WaitCommand(350),
                            new InstantCommand(gripper::open),
                            //new DepositToStateCommand(arm, wrist, gripper, lift,"wallToSpecimen"),
                            new InstantCommand(() -> DepositState = "specimen"),
                            new InstantCommand(() -> killSwitchActive = false)
                    ));
                //.cancelWhenActive(wallToSpecimen);

        new Trigger(() -> driver2.getButton(GamepadKeys.Button.DPAD_RIGHT) && DepositState == "intake")
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(()->lift.PIDEnabled= true),
                        new InstantCommand(vintake::in),
                        intakeToSpecimen,
                        //new DepositToStateCommand(arm, wrist, gripper, lift,"intakeToSpecimen"),
                        new InstantCommand(vintake::stop),
                        new InstantCommand(() -> DepositState = "specimen"),
                        new InstantCommand(() -> killSwitchActive = false)
                ));

        //To hang

            new Trigger(() -> driver2.getButton(GamepadKeys.Button.X))
                    .whenActive(new SequentialCommandGroup(
                            new InstantCommand(()->lift.PIDEnabled= false),
                            new InstantCommand(()-> arm.hang()),
                            //new DepositToStateCommand(arm, wrist, gripper, lift,"initHang"),
                            new InstantCommand(() -> DepositState = "specimen"),
                            new InstantCommand(() -> killSwitchActive = true),
                            new InstantCommand(extendo::in)
                    )).cancelWhenActive(specimenToWall)
                    .cancelWhenActive(intakeToWallWithNothing)
                    .cancelWhenActive(intakeToWallWithSomething)
                    .cancelWhenActive(intakeToBasketHigh)
                    .cancelWhenActive(intakeToBasketLow)
                    .cancelWhenActive(intakeToSpecimen)
                    .cancelWhenActive(intakeToIntake)
                    .cancelWhenActive(basketHighToBasketLow)
                    .cancelWhenActive(basketLowToBasketHigh)
                    .cancelWhenActive(basketToIntake)
                    .cancelWhenActive(basketToSpecimen)
                    .cancelWhenActive(basketToWall)
                    .cancelWhenActive(specimenToBasketLow)
                    .cancelWhenActive(specimenToBasketHigh)
                    .cancelWhenActive(specimenToIntake)
                    .cancelWhenActive(wallToBasketHigh)
                    .cancelWhenActive(wallToBasketLow)
                    .cancelWhenActive(wallToIntake)
                    .cancelWhenActive(wallToSpecimen);

//            new Trigger(()-> driver2.getButton(GamepadKeys.Button.START) && killSwitchActive)
//                    .whenActive(new SequentialCommandGroup(
//                            new InstantCommand(()-> killSwitchPressed =true),
//                            //new InstantCommand(()-> lift.liftFF = 0),
//                            new InstantCommand(()->CommandScheduler.getInstance().reset()),
//                            //new InstantCommand(()->CommandScheduler.getInstance().disable()),
//                            //new InstantCommand(()->CommandScheduler.getInstance().unregisterSubsystem(lift)),
//
//                            new InstantCommand(() -> mBL.setMotorDisable()),
//                            new InstantCommand(() -> mBR.setMotorDisable()),
//                            new InstantCommand(() -> mFL.setMotorDisable()),
//                            new InstantCommand(() -> mFR.setMotorDisable())
//                    ));

        }


        //winch code
        {
            new Trigger(() -> driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) != 0)
                    .whenActive(() -> winch.extend(driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));

            new Trigger(() -> driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) != 0)
                    .whenActive(() -> winch.retract(driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));

            new Trigger(() -> driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) == 0 && driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) == 0)
                    .whenActive(new InstantCommand(winch::stop));
        }



    }

    //this is the main run loop
    public void run() {


//        if (killSwitchPressed){
//
////            sEL.setPosition(BotPositions.EXTENDO_IN);
////            sER.setPosition(BotPositions.EXTENDO_IN);
////            sIG.setPosition(BotPositions.INTAKE_ARM_TRANSFER);
//        }
        //else{
            //super.run() actually runs the triggers in the loop
            super.run();
            //if driver2 is holding down the start button, the hanging state of the lift is triggered
            //lift.hanging(driver2.getButton(GamepadKeys.Button.START));

//        if (extendo.extensionPosition > 0.6) {
//            new SequentialCommandGroup(
//                new InstantCommand(intake::up),
//                new WaitCommand(200),
//                new InstantCommand(extendo::in),
//                new WaitCommand(300),
//                new InstantCommand(intake::transfer)
//                );
//        }

            if(AllianceColor.aColor == "blue"){
                notAColor = "red";
            }
            else if(AllianceColor.aColor == "red"){
                notAColor = "blue";
            }

            //This is kinda redundant as the trigger for the drivers to run the intakeOutCommand also has this as a condition.
            //if the detected color of the intake doesn't match the alliance color, spit out the sample and set the wrongColorIntaked bool to true
//        if (intake.checkColor() == notAColor){
//            new ParallelCommandGroup(
//                    new InstantCommand(()->intake.sIW.setPower(BotPositions.INTAKE_OUT)),
//                    new InstantCommand(()->intake.sIO.setPower(BotPositions.INTAKE_OUT))
//            );
//
//                wrongColorIntaked = true;
//        }

            //these define the left and right trigger values as the real triggers and takes there difference divided by ten
            //as the input of the extendo.update method.
        LeftTrigger = driftLock((float) driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)) / 20;

        RightTrigger = driftLock((float) driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)) / 20;

        Trigger = (LeftTrigger - RightTrigger) / 10;

        //additionally if Trigger gets to large in magnitude it is capped that way the extendo can't be manually launched somewhere crazy.
        if (Trigger > .03) {
            Trigger = .025;
        } else if (Trigger < -.03) {
            Trigger = -.025;
        }

            extendo.update(Trigger);

            //this is just so the manual driving of the lift is based off of the right stick values of gamepad2
            lift.ManualMode(driftLock(gamepad2.left_stick_y), driftLock(gamepad2.right_stick_y));

            //applies stick values to motor variables with cubic scaling
            Rotation = cubicScaling(-gamepad1.right_stick_x)*0.7;
            FB = cubicScaling(gamepad1.left_stick_y);
            LR = cubicScaling(-gamepad1.left_stick_x) * 1.2;

            //defines the powers for the motors based on the stick inputs (trust i've written this so many times)
            double mFLPower = FB + LR + Rotation;
            double mFRPower = FB - LR - Rotation;
            double mBLPower = FB - LR + Rotation;
            double mBRPower = FB + LR - Rotation;

            //actually sets the motor powers
            mFL.setPower(mFLPower * CURRENT_SPEED_MULTIPLIER);
            mFR.setPower(mFRPower * CURRENT_SPEED_MULTIPLIER);
            mBL.setPower(mBLPower * CURRENT_SPEED_MULTIPLIER);
            mBR.setPower(mBRPower * CURRENT_SPEED_MULTIPLIER);

       // drive.updatePoseEstimate();

            //the following is all telemetry for debugging and verifying things in the teleop
            telemetry.addData("RobotState", DepositState);
            //telemetry.addData("IntakeState", intake.checkSample());
            telemetry.addData("AssignedExtensionPosition", Trigger);
            telemetry.addData("ActualExtensionPosition", extendo.sER.getPosition());
            telemetry.addData("checkIntake", vintake.checkSample());
            telemetry.addData("check sample present", vintake.samplePresent);
            telemetry.addData("DetectedColor", vintake.checkColor());
            //telemetry.addData("Blue", intake.checkBlue());
            telemetry.addData("Alliance Color", AllianceColor.aColor);
        telemetry.addData("notAColor", notAColor);
            telemetry.addData("wrongColorDetected", wrongColorIntaked);
           // telemetry.addData("isHanging?", lift.liftHanging);
            //telemetry.addData("LiftAssignedPower", lift.motorPower);
            telemetry.addData("SpeedMultiplyer", CURRENT_SPEED_MULTIPLIER);
            //telemetry.addData("PIDEnabled?", lift.PIDEnabled);
            //telemetry.addData("JoystickPowerInput", lift.joystickPowerInput);
            //telemetry.addData("liftPosition", lift.getCurrentPosition());

            //IMPORTANT: The automatic closing of the gripper is dependent on its sensor always being called
            //just having the verifyGripper() method in a conditional in the objects periodic loop doesn't get it to run continuously
            //thus it needs to be called in the run loop in some way, in this case as telemetry
            telemetry.addData("GripperState", gripper.verifyGripper());

            //telemetry.addData("LiftTopMotorPower", lift.getCurrentMotorPower());
            //telemetry.addData("LiftBottomMotorPower", lift.getCurrentMotorPower());
            //telemetry.addData("LiftTopMotorCurrent", lift.mLT.getCurrent(CurrentUnit.MILLIAMPS));
            //telemetry.addData("LiftBottomMotorCurrent", lift.mLB.getCurrent(CurrentUnit.MILLIAMPS));
            //telemetry.addData("Yellow", intake.checkYellow());
            telemetry.addData("ReadingIntakeRED", vintake.cI.red());//620-650 Yellow 300-400 Red
            telemetry.addData("ReadingIntakeBLUE", vintake.cI.blue());//120-250 Blue
            telemetry.addData("ReadingGripperRED", gripper.cG.red());//620-650 Yellow 300-400 Red
            telemetry.addData("ReadingGripperBLUE", gripper.cG.blue());//120-250 Blue

            //telemetry.addData("ReadingIntake", cI.green());

        telemetry.addData("MotorEncoder",lift.getCurrentPosition());
        telemetry.addData("ShaftEncoder", lift.mLB.getCurrentPosition());

            telemetry.update();

       // }


    }
    //supa cewl cubic scaling method
    //new methods need to be written outside of the run loop since the run loop is a method and there should be no new methods in methods.
    private double cubicScaling(float joystickValue) {
        //store 5% of the joystick value + 95% of the joystick value to the 3rd power
        double v = 0.05 * joystickValue + 0.95 * Math.pow(joystickValue, 3);
        if (joystickValue > 0.02)
            //if the joystick is positive, return positive .1 + the stored value
            return 0.1 + v;
        else if (joystickValue < -0.02)
            //if the joystick is negative, return -.1 plus the stored value
            return -0.1 + v;
        // theres a range where this won't do either, which is a good counter against stick drift (because you can never escape stick drift)
        else
            return 0;
    }
    private double driftLock(float stickValue){
        if(stickValue > 0.02 || stickValue < -.02){
            return stickValue;
        }
        else {
            return  0;
        }
    }
}
