package org.firstinspires.ftc.teamcode.Echo.TeleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Echo.Subsystems.AllianceColor;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Echo.Subsystems.Intake;

@TeleOp(name = "SensorCalibration", group = "AGen1")
public class SensorCalibrationTest extends CommandOpMode {
    Gripper gripper;
    Intake intake;

    @Override
    public void initialize(){
        gripper = new Gripper(hardwareMap);

        intake = new Intake(hardwareMap);
    }
    public void run() {
        super.run();
        gripper.checkColor();

        telemetry.addData("Gripper_Detected_Color", AllianceColor.aColor);
        telemetry.addData("Gripper_Blue_Recording", gripper.cG.blue());
        telemetry.addData("Gripper_Red_Recording", gripper.cG.red());

        telemetry.addData("Intake_detected_red?", intake.checkRed());
        telemetry.addData("Intake_detected_blue?", intake.checkBlue());
        telemetry.addData("Intake_Blue_Recording", intake.cI.blue());
        telemetry.addData("Intake_Red_Recording", intake.cI.red());

        telemetry.update();


    }

}
