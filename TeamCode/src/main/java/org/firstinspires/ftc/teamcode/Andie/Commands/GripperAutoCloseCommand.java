package org.firstinspires.ftc.teamcode.Andie.Commands;



import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Andie.Subsystems.Gripper;

public class GripperAutoCloseCommand extends SequentialCommandGroup { ;

    @SuppressLint("NotConstructor")

    private String botState;

    public GripperAutoCloseCommand (Gripper gripper){
        if (botState == "wall" && gripper.verifyGripper()){
            addCommands(
            new SequentialCommandGroup(
                    new InstantCommand(gripper::close),
                    new WaitCommand(500),
                    new InstantCommand(this::isFinished)
            ));
        }

    }
}
