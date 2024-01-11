// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimeLight extends SubsystemBase {
  NetworkTable table;

  double[] pose;
  NetworkTableEntry pipelineEntry;
  boolean on = false;

  public LimeLight() {
    table = NetworkTableInstance.getDefault().getTable(Constants.Limelight.limelightName); // Set up Network table for Limelight info
    pipelineEntry = table.getEntry("pipeline");  
    pipelineEntry.setNumber(0);  // Set to April tag pipeline.  Should be the only one since no reflective tape this year
    table.getEntry("ledMode").setNumber(1); // Turn LEDs off.  Should be off but just to be safe
  }

  // 3D transform of the camera in the coordinate system of the robot (array (6))
  // a.k.a.: Where the camera is on the field: x, y, z and rotation.
  public double[] getCameraSpace() {
    pose = table.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
    //System.out.println(pose);
    return pose;
  }

  //3D transform of the robot in the coordinate system of the primary in-view AprilTag (array (6)) 
  // a.k.a: the location of center of robot on field: x, y, z and rotation
  // In the limelight config you tell it where the camera is in relation to the center of the robot and it will use that to 
  //   figure out where the center of the robot is on the field based on the Apriltag it sees
  public double[] getTargetSpace() {
    pose = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    //System.out.println(pose);
    return pose;
  }

   // This is a resultTimestamp of when the lat image was captured.
   // If you are getting a picture are 25ms but your limelight subsystem's execute is being called every 50ms. You 
   //  will get a repeat picture and there is no need to process it.
  public double getLatency() {
    return table.getEntry("tl").getDouble(0);
  }
  
  // % of Apriltag the camera can see:  0 - 100
  public double getArea() {
    return table.getEntry("ta").getDouble(0);
  }

 // Return Apriltag number
  public double getApriltagNum() {
    return table.getEntry("tid").getDouble(0);
  }

//  public void setPipeline(String pipeline) {
 //   switch(pipeline) {
 //     case "april":
 //       pipelineEntry.setNumber(0);
//        break;
//      case "tape":
//        pipelineEntry.setNumber(1);
 //       break;
//      default: break;
//    }
//  }

 //Robot transform in field-space. Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
 // This looks to just be getting the rotation (Yaw) of the robot.  It is getting the 6th piece of the array which is the yaw.
 // 0 = x location, 1 = Y location, 2 = z location, 3 = Roll, 4 = Pitch, 5 = Yaw, 6 = total latency (cl+tl)
  public double getRot() {
    return table.getEntry("botpose").getDoubleArray(new double[6])[5];
  }

  // This gets the whole 7 values of the botpose:  Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
  public double[] getBotPose() {
    return table.getEntry("botpose").getDoubleArray(new double[6]);
  }

  //Robot transform in field-space (blue driverstation WPILIB origin). Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
  // So I think this give you your location based on X and Y start is right corner of blue alliance wall.
  // Not sure how this is different from botpose.  One might be better than the other. 
  // If this is blue alliance start then what does botpose start from?  Need to test
  public double[] getBotPoseBlue() {
    return table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
  }

  //Robot transform in field-space (red driverstation WPILIB origin). Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
 public double[] getBotPoseRed() {
    return table.getEntry("botpose_wpired").getDoubleArray(new double[6]);
  }

  
   //Apriltag transform in field-space. Tells you where taget is in relation to robot.
   // Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
 public double[] getTargetPose() {
    return table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // ID of the primary in-view AprilTag
  // This returns true if tag  found and number is between 1 and 16. 16 April tags for 2024 - Crescendo
  public boolean getTargets() {
    //System.out.println(table.getEntry("tid").getDouble(-1));
    if(table.getEntry("tid").getDouble(-1) >= 0 && table.getEntry("tid").getDouble(-1) < 17) {
      return true;
    }
    return false;
  }

  
    // This turns green lights on or off (0 = set based on current pipeline config, 1 = Force off, 2 = Force blink, 3 = force on)
  public void toggleLimelight()
  {
    on = !on;

    if(on)
    {
    table.getEntry("ledMode").setNumber(1);
    }
    else{
      table.getEntry("ledMode").setNumber(3);
    }
  }
}

