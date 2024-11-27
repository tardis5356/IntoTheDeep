package org.firstinspires.ftc.teamcode.Andie.Subsystems;

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
    public void extend(){
        mWR.setPower(1);
        mWL.setPower(1);
    }
    public void retract(){
        mWR.setPower(-1);
        mWL.setPower(-1);
    }
    public void stop(){
        mWR.setPower(0);
        mWL.setPower(0);
    }
}
