package org.firstinspires.ftc.teamcode.Andie.TeleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Andie.Subsystems.Lift;

public class LiftTest extends CommandOpMode {
    private Gamepad aparatus;

    private Lift lift;
    @Override
    public void initialize() {
        aparatus = gamepad1;
        lift = new Lift(hardwareMap);

    }
    public void run() {
        super.run();
        lift.ManualMode(cubicScaling(gamepad2.left_stick_y), gamepad2.right_stick_y);
    }
    private double cubicScaling(float joystickValue) {
        double v = 0.05 * joystickValue + 0.95 * Math.pow(joystickValue, 3);
        if (joystickValue > 0.02)
            return 0.1 + v;
        else if (joystickValue < -0.02)
            return -0.1 + v;
        else
            return 0;
    }
}
