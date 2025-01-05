
package com.ma5951.utils.Utils;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class DriverStationUtil {

    public static Alliance getAlliance() {
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                return Alliance.Blue;
            } else {
                return Alliance.Red;
            }
        }

        return Alliance.Blue;
    }

   



}
