package org.firstinspires.ftc.teamcode.Andie.TeleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Andie.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Extendo;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Andie.Subsystems.Wrist;

@TeleOp (name = "MaxOrbitalPosition")
public class MaxOrbital extends CommandOpMode {

    Arm arm;
    //Lift lift;
    Extendo extendo;
    Intake intake;
    Wrist wrist;

    @Override
    public void initialize() {
        intake = new Intake(hardwareMap);

        //lift
        //lift = new Lift(hardwareMap);

        //wrist
        wrist = new Wrist(hardwareMap);

        //intake
        intake = new Intake(hardwareMap);

        arm = new Arm(hardwareMap);

        extendo = new Extendo(hardwareMap);

        new SequentialCommandGroup(
                new InstantCommand(intake::up),
                new InstantCommand(extendo::out),
                //new LiftToStateCommand(lift, BotPositions.LIFT_TRANSIT,200),
                //new WaitCommand(1000),
                new InstantCommand(arm::basket),
                new InstantCommand(wrist::basket)
        );



    }
    public void run() {
        super.run();
    }

}
