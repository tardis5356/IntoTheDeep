package org.firstinspires.ftc.teamcode.TestBed;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name="LEDTest")
@Config
public class GobildaLEDTest extends LinearOpMode {

    public Servo LED;
    double COLOR = .5;
    @Override

    public void runOpMode() {

        LED = hardwareMap.get(Servo.class, "LED");

        waitForStart();
        while(opModeIsActive()){
            COLOR += gamepad1.left_stick_y/1000;
            if (COLOR < 0){
                COLOR = 0;
            }
            else if (COLOR > 1){
                COLOR = 1;
            }

            LED.setPosition(COLOR);

        }
    }
}
