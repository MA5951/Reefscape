// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

/** Add your docs here. */
public class LoggedPose3d {

    private StructPublisher<Pose3d> loggedPose;
    private Pose3d lastPose;
    private NetworkTable networkTable;


    public LoggedPose3d(String name) {
        networkTable = NetworkTableInstance.getDefault().getTable("/");
        loggedPose = networkTable.getStructTopic(name , Pose3d.struct).publish();
        loggedPose.set(null);
        lastPose = null;
    }

    public void update(Pose3d pose) {
        if (lastPose != pose) {
            loggedPose.set(pose);
            lastPose = pose;
        }   

    }
}
