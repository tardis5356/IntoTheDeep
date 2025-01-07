package org.firstinspires.ftc.teamcode.Echo.TeleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Echo.Commands.LiftToStateCommand;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Extendo;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Winch;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Wrist;

@TeleOp (name = "InspectionTeleOp")
public class MaxOrbital extends CommandOpMode {

    GamepadEx driver1;
    Arm arm;
    Lift lift;
    Extendo extendo;
    Intake intake;
    Wrist wrist;
    Winch winch;

    @Override
    public void initialize() {

        driver1 = new GamepadEx(gamepad1);
        intake = new Intake(hardwareMap);

        //lift
        lift = new Lift(hardwareMap);

        //wrist
        wrist = new Wrist(hardwareMap);

        //intake
        intake = new Intake(hardwareMap);

        arm = new Arm(hardwareMap);

        extendo = new Extendo(hardwareMap);

        winch = new Winch(hardwareMap);

        new Trigger(() -> driver1.getButton(GamepadKeys.Button.A))
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(extendo::out),
                        new WaitCommand(700),
                        new InstantCommand(()->intake.sIG.setPosition(BotPositions.INTAKE_ARM_UP)),
                        new LiftToStateCommand(lift, BotPositions.LIFT_TRANSIT,BotPositions.LIFT_TOLERANCE),
                        new WaitCommand(500),
                        new InstantCommand(arm::basket),
                        new InstantCommand(wrist::basket)
                        //new InstantCommand(winch::extend)
                )
        );



    }
    public void run() {
        super.run();
    }

}
