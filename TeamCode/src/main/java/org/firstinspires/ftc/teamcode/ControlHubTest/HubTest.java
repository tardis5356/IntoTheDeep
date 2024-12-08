package org.firstinspires.ftc.teamcode.ControlHubTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp (name = "HubTest", group = "Tests")
public class HubTest extends LinearOpMode {

    //This program is just to manually test the motor, servo, digital, and I2C ports on hub.
    //Its also an example of a LinearOpMode should that prove useful.

    DcMotor m0, m1, m2, m3;
    Servo s0, s1, s2, s3, s4, s5;
    TouchSensor t0, t1, t2, t3;
    ColorSensor c0, c1, c2, c3;

    @Override
    public void runOpMode(){

        m0 = hardwareMap.get(DcMotor.class, "m0");
        m1 = hardwareMap.get(DcMotor.class, "m1");
        m2 = hardwareMap.get(DcMotor.class, "m2");
        m3 = hardwareMap.get(DcMotor.class, "m3");

        s0 = hardwareMap.get(Servo.class, "s0");
        s1 = hardwareMap.get(Servo.class, "s1");
        s2 = hardwareMap.get(Servo.class, "s2");
        s3 = hardwareMap.get(Servo.class, "s3");
        s4 = hardwareMap.get(Servo.class, "s4");
        s5 = hardwareMap.get(Servo.class, "s5");

        t0 = hardwareMap.get(TouchSensor.class, "t0");
        t1 = hardwareMap.get(TouchSensor.class, "t1");
        t2 = hardwareMap.get(TouchSensor.class, "t2");
        t3 = hardwareMap.get(TouchSensor.class, "t3");

        c0 = hardwareMap.get(ColorSensor.class, "c0");
        c1 = hardwareMap.get(ColorSensor.class, "c1");
        c2 = hardwareMap.get(ColorSensor.class, "c2");
        c3 = hardwareMap.get(ColorSensor.class, "c3");

        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.a){
                m0.setPower(1);
                m1.setPower(1);
                m2.setPower(1);
                m3.setPower(1);
            }

            if(gamepad1.b){
                s0.setPosition(1);
                s1.setPosition(0);
                s2.setPosition(1);
                s3.setPosition(0);
                s4.setPosition(1);
                s5.setPosition(0);
            }

            telemetry.addData("MotorEncoder_0", m0.getCurrentPosition());
            telemetry.addData("MotorEncoder_1", m1.getCurrentPosition());
            telemetry.addData("MotorEncoder_2", m2.getCurrentPosition());
            telemetry.addData("MotorEncoder_3", m3.getCurrentPosition());
            telemetry.addData("Digital_0", t0.isPressed());
            telemetry.addData("Digital_1", t1.isPressed());
            telemetry.addData("Digital_2", t2.isPressed());
            telemetry.addData("Digital_3", t3.isPressed());
            telemetry.addData("I2C_0", c0.blue());
            telemetry.addData("I2C_1", c1.blue());
            telemetry.addData("I2C_2", c2.blue());
            telemetry.addData("I2C_3", c3.blue());
            telemetry.update();
        }
    }
}
