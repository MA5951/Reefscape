// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

/** Add your docs here. */
public class LoggedString {

    private StringPublisher loggedNum;
    private String lastNum;
    private NetworkTable networkTable;


    public LoggedString(String name) {
        networkTable = NetworkTableInstance.getDefault().getTable("/");
        loggedNum = networkTable.getStringTopic(name).publish();
        loggedNum.set("-");
        lastNum = "-";
    }

    public void update(String str) {
        if (lastNum != str) {
            loggedNum.set(str);
            lastNum = str;
        }   

    }
}
