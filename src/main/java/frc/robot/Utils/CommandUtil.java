
package frc.robot.Utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class CommandUtil {

//TODO add comment's
    public static Command instantOf(Runnable toRun) {
        return new InstantCommand(toRun);
    }
    
}
