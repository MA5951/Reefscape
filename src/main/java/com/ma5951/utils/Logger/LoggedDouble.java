// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.Logger;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class LoggedDouble {

    private DoublePublisher loggedNum;
    private double lastNum;
    private NetworkTable networkTable;

    public LoggedDouble(String name) {
        networkTable = NetworkTableInstance.getDefault().getTable("/");
        loggedNum = networkTable.getDoubleTopic(name).publish();
        loggedNum.set(0);
        lastNum = 0;
    }

    public void update(Double num) {
        if (lastNum != num) {
            loggedNum.set(num);
            lastNum = num;
        }   

    }
}
