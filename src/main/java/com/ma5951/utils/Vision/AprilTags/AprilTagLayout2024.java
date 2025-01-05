package com.ma5951.utils.Vision.AprilTags;
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package com.ma5951.utils.Vision;


// import java.io.IOException;
// import java.util.ArrayList;
// import java.util.Arrays;
// import java.util.Comparator;
// import java.util.Comparator;
// import java.util.List;

// import org.opencv.objdetect.Board;

// import com.ma5951.utils.DashBoard.MAShuffleboard;
// import com.ma5951.utils.Logger.LoggedDouble;
// import com.ma5951.utils.Logger.LoggedPose2d;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;



// /** Add your docs here. */
// public class AprilTagLayout2024 {

//     private AprilTagFieldLayout layout;
//     private AprilTag[] aprilTags = new AprilTag[16];
//     private Pose2d[] aprilTagsPoses = new Pose2d[16];
//     private Double maxDetectionDistance = 5d;

//     //This array represents the priority of each tag, higher number here will result at higher priorety when using getBestTag(Pose2d pose)
//     private Double[] tagPriority = new Double[] { 1.7 , 1.7 , 2d , 2d , 1d , 1d , 2d , 2d , 1.7 , 1.7 , 1d , 1d , 1d , 1d , 1d , 1d };

//     private LoggedDouble rating1;
//     private LoggedDouble rating2;
//     private LoggedDouble rating3;
//     private LoggedDouble rating4;
//     private LoggedDouble rating5;
//     private LoggedDouble rating6;
//     private LoggedDouble rating7;
//     private LoggedDouble rating8;
//     private LoggedDouble rating9;
//     private LoggedDouble rating10;
//     private LoggedDouble rating11;
//     private LoggedDouble rating12;
//     private LoggedDouble rating13;
//     private LoggedDouble rating14;
//     private LoggedDouble rating15;
//     private LoggedDouble rating16;

//     private LoggedPose2d tag1Pose;
//     private LoggedPose2d tag2Pose;
//     private LoggedPose2d tag3Pose;
//     private LoggedPose2d tag4Pose;
//     private LoggedPose2d tag5Pose;
//     private LoggedPose2d tag6Pose;
//     private LoggedPose2d tag7Pose;
//     private LoggedPose2d tag8Pose;
//     private LoggedPose2d tag9Pose;
//     private LoggedPose2d tag10Pose;
//     private LoggedPose2d tag11Pose;
//     private LoggedPose2d tag12Pose;
//     private LoggedPose2d tag13Pose;
//     private LoggedPose2d tag14Pose;
//     private LoggedPose2d tag15Pose;
//     private LoggedPose2d tag16Pose;

//     private LoggedPose2d bestTag1;
//     private LoggedPose2d bestTag2;
//     private LoggedPose2d bestTag3;

    
    

//     public AprilTagLayout2024() {
//         try {
//             layout = AprilTagFieldLayout.loadFromResource("/edu/wpi/first/apriltag/2024-crescendo.json");
//         } catch (IOException e) {
//             e.printStackTrace();
//             System.out.println("Couldent load apriltag map");
//         }
//         //Copy apriltags data to aprilTags arry
//         for (int i = 0; i < 16; i++) {
//             aprilTags[i] = new AprilTag( i + 1 ,layout.getTagPose(i + 1).get());
//         }
//         //Copy aprilTags locations to aprilTagsPoses arry
//         for (int i = 0; i < 16; i++) {
//             aprilTagsPoses[i] = aprilTags[i].getPose2D();
//         }
        
//         rating1 = new LoggedDouble("/AprilTagVision/Tag Raiting/1");
//         rating2 = new LoggedDouble("/AprilTagVision/Tag Raiting/2");
//         rating3 = new LoggedDouble("/AprilTagVision/Tag Raiting/3");
//         rating4 = new LoggedDouble("/AprilTagVision/Tag Raiting/4");
//         rating5 = new LoggedDouble("/AprilTagVision/Tag Raiting/5");
//         rating6 = new LoggedDouble("/AprilTagVision/Tag Raiting/6");
//         rating7 = new LoggedDouble("/AprilTagVision/Tag Raiting/7");
//         rating8 = new LoggedDouble("/AprilTagVision/Tag Raiting/8");
//         rating9 = new LoggedDouble("/AprilTagVision/Tag Raiting/9");
//         rating10 = new LoggedDouble("/AprilTagVision/Tag Raiting/10");
//         rating11 = new LoggedDouble("/AprilTagVision/Tag Raiting/11");
//         rating12 = new LoggedDouble("/AprilTagVision/Tag Raiting/12");
//         rating13 = new LoggedDouble("/AprilTagVision/Tag Raiting/13");
//         rating14 = new LoggedDouble("/AprilTagVision/Tag Raiting/14");
//         rating15 = new LoggedDouble("/AprilTagVision/Tag Raiting/15");
//         rating16 = new LoggedDouble("/AprilTagVision/Tag Raiting/16");


//         tag1Pose = new LoggedPose2d("/AprilTagVision/Tag Poses/1");
//         tag2Pose = new LoggedPose2d("/AprilTagVision/Tag Poses/2");
//         tag3Pose = new LoggedPose2d("/AprilTagVision/Tag Poses/3");
//         tag4Pose = new LoggedPose2d("/AprilTagVision/Tag Poses/4");
//         tag5Pose = new LoggedPose2d("/AprilTagVision/Tag Poses/5");
//         tag6Pose = new LoggedPose2d("/AprilTagVision/Tag Poses/6");
//         tag7Pose = new LoggedPose2d("/AprilTagVision/Tag Poses/7");
//         tag8Pose = new LoggedPose2d("/AprilTagVision/Tag Poses/8");
//         tag9Pose = new LoggedPose2d("/AprilTagVision/Tag Poses/9");
//         tag10Pose = new LoggedPose2d("/AprilTagVision/Tag Poses/10");
//         tag11Pose = new LoggedPose2d("/AprilTagVision/Tag Poses/11");
//         tag12Pose = new LoggedPose2d("/AprilTagVision/Tag Poses/12");
//         tag13Pose = new LoggedPose2d("/AprilTagVision/Tag Poses/13");
//         tag14Pose = new LoggedPose2d("/AprilTagVision/Tag Poses/14");
//         tag15Pose = new LoggedPose2d("/AprilTagVision/Tag Poses/15");
//         tag16Pose = new LoggedPose2d("/AprilTagVision/Tag Poses/16");

//         tag1Pose.update(aprilTagsPoses[0]);
//         tag2Pose.update(aprilTagsPoses[1]);
//         tag3Pose.update(aprilTagsPoses[2]);
//         tag4Pose.update(aprilTagsPoses[3]);
//         tag5Pose.update(aprilTagsPoses[4]);
//         tag6Pose.update(aprilTagsPoses[5]);
//         tag7Pose.update(aprilTagsPoses[6]);
//         tag8Pose.update(aprilTagsPoses[7]);
//         tag9Pose.update(aprilTagsPoses[8]);
//         tag10Pose.update(aprilTagsPoses[9]);
//         tag11Pose.update(aprilTagsPoses[10]);
//         tag12Pose.update(aprilTagsPoses[11]);
//         tag13Pose.update(aprilTagsPoses[12]);
//         tag14Pose.update(aprilTagsPoses[13]);
//         tag15Pose.update(aprilTagsPoses[14]);
//         tag16Pose.update(aprilTagsPoses[15]);

//         bestTag1 = new LoggedPose2d("/AprilTagVision/Best Tags/1");
//         bestTag2 = new LoggedPose2d("/AprilTagVision/Best Tags/2");
//         bestTag3 = new LoggedPose2d("/AprilTagVision/Best Tags/3");



//         //printTagsData(); //Uncommet if you want to print tags data

//     }


//     public void printTagsData() {
//         for (int i = 0; i < 16; i++) {
//             System.out.println(aprilTags[i].toString());
//         }
//     }

//     public AprilTag getTag(int id) {
//         return aprilTags[ id-1 ];
//     }


//     public AprilTag getClosesTag(Pose2d pose) {
//         int index = -1;
//         for (int i = 0; i < aprilTagsPoses.length; i++) {
//             if (aprilTagsPoses[i].equals(pose.nearest(Arrays.asList(aprilTagsPoses)))) {
//                 index = i;
//                 break;
//             }
//         }
//         return aprilTags[index];
//     }

//     public AprilTag getBestTag(Pose2d pose) {
//         double[] tagsRaitings = new double[16];
//         int bestTag = 0;
//         for (int i = 0; i < aprilTagsPoses.length ; i++) {
//             tagsRaitings[i] = getDistance(pose, aprilTagsPoses[i]);
//             tagsRaitings[i] = tagsRaitings[i] / tagPriority[i];
//             if (getDistance(pose, aprilTagsPoses[i]) > maxDetectionDistance) {
//                 tagsRaitings[i] += 10;
//             }
//         }

//         rating1.update(tagsRaitings[0]);
//         rating2.update(tagsRaitings[1]);
//         rating3.update(tagsRaitings[2]);
//         rating4.update(tagsRaitings[3]);
//         rating5.update(tagsRaitings[4]);
//         rating6.update(tagsRaitings[5]);
//         rating7.update(tagsRaitings[6]);
//         rating8.update(tagsRaitings[7]);
//         rating9.update(tagsRaitings[8]);
//         rating10.update(tagsRaitings[9]);
//         rating11.update(tagsRaitings[10]);
//         rating12.update(tagsRaitings[11]);
//         rating13.update(tagsRaitings[12]);
//         rating14.update(tagsRaitings[13]);
//         rating15.update(tagsRaitings[14]);
//         rating16.update(tagsRaitings[15]);


//         Integer[] indices = new Integer[tagsRaitings.length];
//         for (int i = 0; i < tagsRaitings.length; i++) {
//             indices[i] = i;
//         }
//         Arrays.sort(indices, Comparator.comparingDouble(i -> tagsRaitings[i]));
//         Integer[] lowestIndices = Arrays.copyOfRange(indices, 0, 3);

//         bestTag1.update(aprilTagsPoses[lowestIndices[0]]);
//         bestTag2.update(aprilTagsPoses[lowestIndices[1]]);
//         bestTag3.update(aprilTagsPoses[lowestIndices[2]]);

//         return aprilTags[lowestIndices[0]];

//     }

//     public double getDistance(Pose2d pose1  , Pose2d pose2) {
//         return Math.abs(pose1.getTranslation().getDistance(pose2.getTranslation()));
//     }


//     // public double getAngleToTag(Pose2d pose , AprilTag tag) {
//     //     double xDis = Math.abs(pose.getX() - tag.getPose2D().getX());
//     //     double yDis = Math.abs(pose.getY() - tag.getPose2D().getY());
//     //     double angle = Math.atan(yDis / xDis);
//     //     // if (DriverStation.getAlliance().isPresent()) {
//     //     //     angle = DriverStation.getAlliance().get() == Alliance.Blue ?
//     //     //     -(angle - Math.PI) : angle;
//     //     // }
//     //     if (pose.getY() > tag.getY()) {
//     //     angle = -angle;
//     //     }
//     //     Math.                                                                      
//     //     return angle;
//     // }

//     public Rotation2d calculateAngleToTag(Pose2d robotPose, AprilTag tag) {
//         // Get the translations from the poses
//         Translation2d robotTranslation = robotPose.getTranslation();
//         Translation2d tagTranslation = tag.getPose2D().getTranslation();

//         // Calculate the difference in positions
//         Translation2d difference = tagTranslation.minus(robotTranslation);

//         // Calculate the angle to the tag relative to the field
//         Rotation2d angleToTag = new Rotation2d(Math.atan2(difference.getY(), difference.getX()));

//         return angleToTag;
//     }


//     public double calculateAngleRelativeToRobot(Pose2d robotPose, AprilTag tag) {
//         Rotation2d fieldRelativeAngle = calculateAngleToTag(robotPose, tag);
//         Rotation2d robotRotation = robotPose.getRotation();
        
//         // Calculate the angle relative to the robot's current orientation
//         double angleRelativeToRobot = fieldRelativeAngle.minus(robotRotation).getDegrees();

//         // Normalize the angle to be within the range [0, 360)
//         angleRelativeToRobot = (angleRelativeToRobot + 360) % 360;

//         return angleRelativeToRobot;
//     }

//     // public double calculateAngleRelativeToRobot(Pose2d robotPose, AprilTag tag) {
//     //     Rotation2d fieldRelativeAngle = getAngleToTag(robotPose, tag);
//     //     Rotation2d robotRotation = robotPose.getRotation();
//     //     Rotation2d angleRelativeToRobot = fieldRelativeAngle.minus(robotRotation);

//     //     // Convert the angle to degrees
//     //     double angleInDegrees = Math.toDegrees(angleRelativeToRobot.getDegrees());

//     //     // Adjust the angle to be in the range [0, 360)
//     //     if (angleInDegrees < 0) {
//     //         angleInDegrees += 360;
//     //     }

//     //     return angleInDegrees;
//     // }



    
// }
