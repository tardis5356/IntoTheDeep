package org.firstinspires.ftc.teamcode.DemoBots.optimus.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.DemoBots.optimus.Optimus_subsystems.OptimusDrive;

public class OptimusTurnIMUCommand extends CommandBase {
    private OptimusDrive drive;

    private int targetAngle, tolerance;

    private float angle;

    private boolean inTolerance = false;

    public OptimusTurnIMUCommand(OptimusDrive drive, int targetAngle, int tolerance) {
        this.drive = drive;
        this.targetAngle = targetAngle;
        this.tolerance = tolerance;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (AngleUnit.normalizeDegrees(drive.getIMUAngle() - targetAngle) > tolerance / 2) {
            drive.slowTurnRight();
        } else if (AngleUnit.normalizeDegrees(drive.getIMUAngle() - targetAngle) < -tolerance / 2) {
            drive.slowTurnLeft();
        } else {
            inTolerance = true;
        }
    }

    public float getError(){
        return drive.getIMUAngle() - targetAngle;
    }

    @Override
    public boolean isFinished() { // returns true when finished
        return inTolerance;
    }

    @Override
    public void end(boolean interrupted) {
//        lift.stop();
        drive.stopDriving();
    }
}
