package com.ma5951.utils.Vision.AprilTags;
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package com.ma5951.utils.Vision;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation3d;

// /** Add your docs here. */
// public class AprilTag {
//     private Pose3d tagPose;
//     private Rotation3d tagRotation;
//     private int id;

//     //All locations are rounded ***************** Values are not accurate

//     public AprilTag(int ID , double X , double Y , double Z , double Yaw) {
//         id = ID;
//         tagRotation = new Rotation3d(0, 0, Yaw );
//         tagPose = new Pose3d( X , Y , Z , tagRotation);
//     }

//     public AprilTag(int ID , double X , double Y , double Yaw) {
//         id = ID;
//         tagRotation = new Rotation3d(0, 0, Yaw );
//         tagPose = new Pose3d( X , Y , 0, tagRotation);
//     }

//     public AprilTag(int ID , Pose3d pose) {
//         tagPose = pose;
//         id = ID;
//     }

//     public double getX() {
//         return tagPose.getX();
//     }

//     public double getY() {
//         return tagPose.getY();
//     }

//     public double getZ() {
//         return tagPose.getZ();
//     }

//     public double getYaw() {
//         return tagPose.getRotation().getZ();
//     }

//     public Pose2d getPose2D() {
//         return tagPose.toPose2d();
//     }

//     public Pose3d getPose3D() {
//         return tagPose;
//     }

//     public int getID() {
//         return id;
//     }

//     @Override
//     public String toString() {
//         return "ID: " + getID() + " X: " + getX() +" Y: "+ getY() +" Z: "+ getZ() +"Yaw: "+getYaw();
//     }
// }
