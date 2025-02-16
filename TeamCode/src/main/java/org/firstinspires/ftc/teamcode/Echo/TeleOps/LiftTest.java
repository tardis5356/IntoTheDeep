package org.firstinspires.ftc.teamcode.Echo.TeleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Echo.Commands.LiftToStateCommand;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Wrist;

//@Disabled
@TeleOp(name = "LiftTest")
public class LiftTest extends CommandOpMode {
    private GamepadEx aparatus;

    private Lift lift;
    private Arm arm;
    private Wrist wrist;
    private Gripper gripper;

    @Override
    public void initialize() {
        aparatus = new GamepadEx(gamepad1);
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);
        gripper = new Gripper(hardwareMap);
        wrist = new Wrist(hardwareMap);

        new Trigger(() -> aparatus.getButton(GamepadKeys.Button.DPAD_UP))
                .whenActive( new SequentialCommandGroup(
                                new InstantCommand(()->lift.PIDEnabled = true),
                                new LiftToStateCommand(lift, BotPositions.LIFT_BASKET_HIGH, BotPositions.LIFT_TOLERANCE)
                        )
                );
        new Trigger(() -> aparatus.getButton(GamepadKeys.Button.DPAD_LEFT))
                .whenActive(new LiftToStateCommand(lift, BotPositions.LIFT_BASKET_LOW, BotPositions.LIFT_TOLERANCE));
        new Trigger(() -> aparatus.getButton(GamepadKeys.Button.DPAD_DOWN))
                .whenActive(new LiftToStateCommand(lift, BotPositions.LIFT_TRANSIT, BotPositions.LIFT_TOLERANCE));

        new Trigger(() -> aparatus.getButton(GamepadKeys.Button.DPAD_RIGHT))
                .whenActive( new SequentialCommandGroup(
                            new InstantCommand(()->lift.PIDEnabled = true),
                            new LiftToStateCommand(lift, BotPositions.LIFT_WALL, BotPositions.LIFT_TOLERANCE)
                        )
                );

        new Trigger(() -> aparatus.getButton(GamepadKeys.Button.A))
                .whenActive(new LiftToStateCommand(lift, BotPositions.LIFT_INTAKE, BotPositions.LIFT_TOLERANCE));
        new Trigger(() -> aparatus.getButton(GamepadKeys.Button.B))
                .whenActive(new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH, BotPositions.LIFT_TOLERANCE));

        new Trigger(() -> aparatus.getButton(GamepadKeys.Button.X))
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(arm::specimen),
                        new InstantCommand(wrist::specimen),
                        new InstantCommand(gripper::close)
                ));

        new Trigger(() -> aparatus.getButton(GamepadKeys.Button.Y))
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(arm::specimenHang),
                        new WaitCommand(750),
                        new InstantCommand(gripper::open)
                ));
        new Trigger(() -> aparatus.getButton(GamepadKeys.Button.RIGHT_BUMPER))
                .whenActive(new SequentialCommandGroup(

                        new InstantCommand(gripper::open)
                ));

//        new Trigger(() -> aparatus.getButton(GamepadKeys.Button.DPAD_DOWN))
//                .whenActive(new LiftToStateCommand(lift, BotPositions.LIFT_BASKET_LOW, 1));
//        new Trigger(() -> aparatus.getButton(GamepadKeys.Button.DPAD_LEFT))
//                .whenActive(new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_HIGH, 1));
//        new Trigger(() -> aparatus.getButton(GamepadKeys.Button.DPAD_RIGHT))
//                .whenActive(new LiftToStateCommand(lift, BotPositions.LIFT_SPECIMEN_LOW, 1));
//        new Trigger(() -> aparatus.getButton(GamepadKeys.Button.LEFT_BUMPER))
//                .whenActive(new LiftToStateCommand(lift, BotPositions.LIFT_WALL, 1));
//        new Trigger(() -> aparatus.getButton(GamepadKeys.Button.RIGHT_BUMPER))
//                .whenActive(new LiftToStateCommand(lift, BotPositions.LIFT_TRANSIT, 1));
    }
    public void run() {
        super.run();

        lift.ManualMode(aparatus.getLeftY(), aparatus.getRightY());

        telemetry.addData("LeftStick", aparatus.getLeftY());
        telemetry.addData("RightStick", aparatus.getRightY());
        telemetry.addData("liftPosition", lift.getCurrentPosition());
        telemetry.addData("TargetPosition", lift.getTargetPosition());
        telemetry.addData("PID power", lift.getCurrentPID());
        telemetry.addData("LiftAssignedPower", lift.motorPower);
        telemetry.update();


    }
//    private double cubicScaling(float joystickValue) {
//        double v = 0.05 * joystickValue + 0.95 * Math.pow(joystickValue, 3);
//        if (joystickValue > 0.02)
//            return 0.1 + v;
//        else if (joystickValue < -0.02)
//            return -0.1 + v;
//        else
//            return 0;
//    }
}
