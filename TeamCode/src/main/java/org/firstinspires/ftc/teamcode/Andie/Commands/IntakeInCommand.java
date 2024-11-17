package org.firstinspires.ftc.teamcode.Andie.Commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Andie.Subsystems.Intake;



public class IntakeInCommand extends SequentialCommandGroup {


    public IntakeInCommand(Intake intake) {
<<<<<<< HEAD
        addCommands(
                new InstantCommand(intake::intakeIn)
        );
=======
        if(intake.checkIntake() == false){
            addCommands(
                    new InstantCommand(intake::intakeIn)
            );
        }
        else if (intake.checkIntake() == true){
            addCommands(
                    new InstantCommand(intake::intakeStop)
            );
        }

>>>>>>> origin/master
    }
}
