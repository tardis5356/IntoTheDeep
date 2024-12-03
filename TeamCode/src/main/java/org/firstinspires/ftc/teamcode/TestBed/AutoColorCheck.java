package org.firstinspires.ftc.teamcode.TestBed;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Andie.Subsystems.AllianceColor;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Gripper;

@Autonomous(name = "AutoColorCheck")
public class AutoColorCheck extends OpMode {

    Gripper gripper;

    @Override
    public void init() {
        gripper = new Gripper(hardwareMap);
    }
    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        gripper.checkColor();
    }

    @Override
    public void loop() {
        telemetry.addData("detectedColor", AllianceColor.aColor);
        telemetry.addData("Blue", gripper.cG.blue());
        telemetry.addData("Red", gripper.cG.red());
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}
