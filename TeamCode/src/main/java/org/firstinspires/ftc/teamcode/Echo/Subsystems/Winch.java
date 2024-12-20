package org.firstinspires.ftc.teamcode.Echo.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Winch extends SubsystemBase {
    private DcMotorEx mWL, mWR;

    public Winch(HardwareMap hardwareMap){
        mWL = hardwareMap.get(DcMotorEx.class, "mWL");
        mWR = hardwareMap.get(DcMotorEx.class, "mWR");

        mWR.setDirection(DcMotorSimple.Direction.REVERSE);

        mWL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mWR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void periodic(){

    }
    public void extend(double lTrigger){
        mWR.setPower(-lTrigger);
        mWL.setPower(-lTrigger);
    }
    public void retract(double rTrigger){
        mWR.setPower(rTrigger);
        mWL.setPower(rTrigger);
    }
    public void stop(){
        mWR.setPower(BotPositions.WHINCH_FF);
        mWL.setPower(BotPositions.WHINCH_FF);
    }
}
