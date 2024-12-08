package org.firstinspires.ftc.teamcode.TestBed;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Andie.Subsystems.AllianceColor;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Gripper;
@Disabled
@TeleOp (name = "TeleOpColorVerification")
public class TeleOpColorVerification extends CommandOpMode {
    Gripper gripper;
    @Override
    public void initialize(){
        gripper = new Gripper(hardwareMap);
    }
    public void run() {
        super.run();
        telemetry.addData("AllianceColor", AllianceColor.aColor);
        telemetry.addData("detectedBlue", gripper.cG.blue());
        //telemetry.addData("detectedRed", gripper.cG.red());
        telemetry.update();
    }
}
