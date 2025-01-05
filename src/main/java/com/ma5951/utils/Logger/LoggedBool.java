// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.Logger;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class LoggedBool {

    private BooleanPublisher loggedBool;
    private boolean lastBool;
    private NetworkTable networkTable;


    public LoggedBool(String name) {
        networkTable = NetworkTableInstance.getDefault().getTable("/");
        loggedBool = networkTable.getBooleanTopic(name).publish();
        loggedBool.set(false);
        lastBool = false;
    }

    public void update(Boolean bool) {
        if (lastBool != bool) {
            loggedBool.set(bool);
            lastBool = bool;
        }   

    }
}
