
package frc.robot.Utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class CommandUtil {


    /** Function used to create an instatn command from a runable
     * 
     * @param toRun Runbale To Run
     */
    public static Command instantOf(Runnable toRun) {
        return new InstantCommand(toRun);
    }
    
}