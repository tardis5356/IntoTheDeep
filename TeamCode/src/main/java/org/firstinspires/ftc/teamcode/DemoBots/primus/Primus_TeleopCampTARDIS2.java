package org.firstinspires.ftc.teamcode.DemoBots.primus;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
@TeleOp(name = "Primus_TeleOp_CycleNo.2", group="demo")
public class Primus_TeleopCampTARDIS2 extends LinearOpMode {    // LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    double zeroPosition = 0;
    boolean encoderReset = false;
    double armPosition;
    double PosDiff;
    boolean FarForward;
    boolean FarBack;
    double vArmPower;
    TouchSensor ArmLim;

    DcMotor mBL;//Back left
    DcMotor mBR;
    DcMotor mFL;
    DcMotor mFR;//Front right


    //connected to sparkMini controller don't have enough ports
    CRServo mArm;




    Servo sL;
    Servo sR;

    @Override
    public void runOpMode() {

        mBL = hardwareMap.dcMotor.get("mBL");//Back left
        mBR = hardwareMap.dcMotor.get("mBR");
        mFL = hardwareMap.dcMotor.get("mFL");
        mFR = hardwareMap.dcMotor.get("mFR");//Front right


        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mBL.setDirection(DcMotor.Direction.FORWARD);
        mBR.setDirection(DcMotor.Direction.REVERSE);
        mFR.setDirection(DcMotor.Direction.REVERSE);
        mFL.setDirection(DcMotor.Direction.REVERSE);

        sL = hardwareMap.servo.get("sL");
        sR = hardwareMap.servo.get("sR");
        mArm = hardwareMap.crservo.get("mArm");

        ArmLim = hardwareMap.get(TouchSensor.class, "aTouch");

        waitForStart();

        while (opModeIsActive()) {

            //Gamepad 1 Variablesz
            double leftY1 = gamepad1.left_stick_y;
            double rightX1 = gamepad1.right_stick_x;

            //Gamepad 2 Variables
            double leftY2 = gamepad2.left_stick_y;
            double rightTrigger2 = gamepad2.right_trigger;
            double leftTrigger2 = gamepad2.left_trigger;
            double rightY2 = gamepad2.right_stick_y;
            boolean aButton = gamepad2.a;
            boolean bButton = gamepad2.b;
            vArmPower = rightY2;

            //Drivetrain controls
            //TODO write drivetrain code using joysticks
            mBL.setPower();
            mBR.setPower();
            mFL.setPower();
            mFR.setPower();

            //TODO write Arm code with limits using limit switch and far foward and far back variables

            if(ArmLim.isPressed() ==  || (FarForward ==  && FarBack == )){
                mArm.setPower(-vArmPower);

            }
            if(ArmLim.isPressed() == ){
                armPosition = 1000;
                PosDiff = armPosition - mBR.getCurrentPosition();
                FarForward = true;
                if(rightY2 > 0){
                    vArmPower = (0);
                }

                mArm.setPower(-vArmPower);
            }
            if(armPosition < 2000 && FarForward == ){
                mArm.setPower(-vArmPower);
            }
            if(armPosition >= 2000){
                FarForward = ;
            }
            if(armPosition >= 5000 || FarBack == ){
                FarBack = ;
                mArm.setPower(-1);
            }
            if(armPosition >= 4000){
                FarBack = ;
            }



            //TODO write code for the grippers with any button
            //sR open = 0.6, close = 0.25
            //sL open = 0.35, close = 0.75

            if (/*button*/ != 0) {
                sR.setPosition(/*Position*/);
            } else {
                sR.setPosition(/*Position*/);
            }
            if (/*button*/ != 0) {
                sL.setPosition(/*Position*/);
            } else {
                sL.setPosition(/*Position*/);
            }

            armPosition = mBR.getCurrentPosition() + PosDiff;

//            telemetry.addData("armLimit", armLimit.getVoltage());
            telemetry.addData("arm power", mArm.getPower());
            telemetry.addData("true arm position", mBR.getCurrentPosition());
            telemetry.addData("zeroPosition", zeroPosition);
            telemetry.addData("ArmPosition", armPosition);
            telemetry.addData("Position Diff", PosDiff);
            telemetry.addData("touchSensor", ArmLim.isPressed());
            telemetry.addData("FarForward", FarForward);
            telemetry.addData("FarBack", FarBack);
            telemetry.update();




        }


    }


}