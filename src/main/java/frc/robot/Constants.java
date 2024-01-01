// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.CANifier;
//import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
//import com.pathplanner.lib.util.PIDConstants;
//import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;



/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    public static final int LEFT_FRONT_SPEED_MOTOR_ID = 10;
    public static final int LEFT_FRONT_DIRECTION_MOTOR_ID = 11;
    public static final int RIGHT_FRONT_SPEED_MOTOR_ID = 20;
    public static final int RIGHT_FRONT_DIRECTION_MOTOR_ID = 21;
    public static final int LEFT_BACK_SPEED_MOTOR_ID = 30;
    public static final int LEFT_BACK_DIRECTION_MOTOR_ID = 31;
    public static final int RIGHT_BACK_SPEED_MOTOR_ID = 40;
    public static final int RIGHT_BACK_DIRECTIION_MOTOR_ID = 41;

    //Absolute Encoders
    public static final int LEFT_FRONT_CANCODER_ID = 12;   
    public static final int RIGHT_FRONT_CANCODER_ID = 22;
    public static final int LEFT_BACK_CANCODER_ID = 32;
    public static final int RIGHT_BACK_CANCODER_ID = 42;

    public static final int CANIFIER_ID = 3;

  }

  public static final int PIGEON_ID = 1;

  public static final double LEFT_FRONT_RADIAN_OFFSET = Math.toRadians(18.457); //18.6328125 //285.908 //-1.318; //-0.023003;
  public static final double RIGHT_FRONT_RADIAN_OFFSET = Math.toRadians(144.932); //143.701171875 //349.198 //-10.898; //2.021; //0.035273;
  public static final double LEFT_BACK_RADIAN_OFFSET = Math.toRadians(10.283); //10.4589 //88.594 //88.682; //-1.143; //-0.019949;
  public static final double RIGHT_BACK_RADIAN_OFFSET = Math.toRadians(147.92); //149.326217//202.236 //562.412; //-2.812; //-0.019949;
 

  public static final boolean LEFT_FRONT_SPEED_IS_REVERSED = true; //false; //true;
  public static final boolean RIGHT_FRONT_SPEED_IS_REVERSED = false; //true; //false;
  public static final boolean LEFT_BACK_SPEED_IS_REVERSED = true; //false; //true;
  public static final boolean RIGHT_BACK_SPEED_IS_REVERSED = false; //true; //false;

  // when changed to true, driving forward and backward are okay. 
  // BUT Left and right are reveresed 
  // AND rotating does NOT work.  Wheels face inward
  public static final boolean LEFT_FRONT_DIRECTION_IS_REVERSED = false;
  public static final boolean RIGHT_FRONT_DIRECTION_IS_REVERSED = false;
  public static final boolean LEFT_BACK_DIRECTION_IS_REVERSED = false;
  public static final boolean RIGHT_BACK_DIRECTION_IS_REVERSED = false;

  //Must stay true or offset angle will be the wrong direction and wheels will not be straignt on startup and driving
  public static final boolean LEFT_FRONT_CANCODER_IS_REVERSED = true; // false;
  public static final boolean RIGHT_FRONT_CANCODER_IS_REVERSED = true; // false;
  public static final boolean LEFT_BACK_CANCODER_IS_REVERSED = true; // false;
  public static final boolean RIGHT_BACK_CANCODER_IS_REVERSED = true; // false;

  public static final double wheelDiameter = Units.inchesToMeters(3.93);  // 4 inches =  .1016 Meters  // Orig was 3.75
  public static final double speedMotorGearRatio  =  (1 / 6.75);    //.148148
  public static final double directionMotorGearRatio =  (1 / 21.4286);   //.04667
  
  // The speed encoder to rotation does not impact teleop just Auto
  public static final double speedEncoderRotationToMeter = speedMotorGearRatio * Math.PI * wheelDiameter;  // Orig - fast and goes way to far in auto.  Does not impact tele
  public static final double speedEncoderRPMToMPS = speedEncoderRotationToMeter / 60;
  
  //Changes to direction encoder do impact tele and Auto
  public static final double directionEncoderRotationToRadian = directionMotorGearRatio * 2 * Math.PI;
  public static final double directionEncoderRPMToRadsPS = directionEncoderRotationToRadian / 60;
  
  
  public static final double kPTurning = 0.15; //what

    ///**** NOTE: if physicalMaxSpeedMPS is too low the auto speeds are crazy fast and distances are off. .005 was NOT GOOD */
  public static final double physicalMaxSpeedMPS = 5; // 0.005; //0.07 before fly-in //is this a double? what should the value be?
  
  /** This impacts seed fo spin/turning of robot. */
  public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI; // 0.007 * 2 * Math.PI; //0.02 before flyin

  public static final double kTeleDriveMaxSpeedMetersPerSecond = physicalMaxSpeedMPS / 1.5;
  public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 1.5;
  public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 1.5;
  public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 4;

   // Start - Created from running Sysid and running test on 12-16-23
  public static final double ksvolts = 0.11307;
  public static final double kvVoltSecondsPerMeter = 2.7105;
  public static final double kaVoltSecondsSquarePerMeter = 0.065426;
  public static final double kPDriveVel = 0.010885;

  //////////// Trying path planner 12-16-23
  public static final class Swerve {
    //public static final Translation2d flModuleOffset = new Translation2d(0.3175, 0.3175);
    public static final Translation2d flModuleOffset = new Translation2d(kWheelBase / 2, kTrackWidth / 2);
    public static final Translation2d frModuleOffset = new Translation2d(0.3175, -0.3175);
    public static final Translation2d blModuleOffset = new Translation2d(-0.3175, 0.3175);
    public static final Translation2d brModuleOffset = new Translation2d(-0.3175, -0.3175);

    public static final double maxModuleSpeed = 1; // orig 4.5 M/S Max speed for auto

  
  public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
      new PIDConstants(2, 1, 0), // Translation constants 
      new PIDConstants(2, 0, 0), // Rotation constants 
      maxModuleSpeed, 
      flModuleOffset.getNorm(), // Drive base radius (distance from center to furthest module) 
      new ReplanningConfig()
    );
  }
  //////////////////////////////////////////

 // END - Created from running Sysid and running test on 12-16-23

  /* public static final double kTeleDriveMaxAccelerationMetersPerSecond = physicalMaxSpeedMPS / 3;
  public static final double kTeleDriveMaxAngularAccelerationRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4; */

  public static final double kTrackWidth = Units.inchesToMeters(24.5); //distance between left and right wheels
  public static final double kWheelBase = Units.inchesToMeters(24.5); //distance between front and back wheels
  public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
    //new Translation2d(kWheelBase / 2, kTrackWidth / 2),     // Front Left wheel position from center of robot
    //new Translation2d(kWheelBase / 2, -kTrackWidth / 2),    // Front Right wheel postion
    //new Translation2d(-kWheelBase / 2, kTrackWidth / 2),    // Back Left wheel postion
    //new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)    // Back Right wheel position 

     //orig values:
    new Translation2d(kWheelBase / 2, -kTrackWidth / 2),   // Front Right wheel position from center of robot
    new Translation2d(kWheelBase / 2, kTrackWidth / 2),    // Front Left wheel postion
    new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),  // Back Right wheel postion
    new Translation2d(-kWheelBase / 2, kTrackWidth / 2)    // Back Left wheel position 
  );

  public static final SwerveDriveKinematics kDriveKinematicsAuto = new SwerveDriveKinematics(
    new Translation2d(kWheelBase / 2, kTrackWidth / 2),     // Front Left wheel position from center of robot
    new Translation2d(kWheelBase / 2, -kTrackWidth / 2),    // Front Right wheel postion
    new Translation2d(-kWheelBase / 2, kTrackWidth / 2),    // Back Left wheel postion
    new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)    // Back Right wheel position

  );

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1; // physicalMaxSpeedMPS - too slow. Jumps arounds; //3 - too fast; // Constants.physicalMaxSpeedMPS / 10; //4;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1; //.001; // .1; //.25; //1; //= 3;

    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;  // Constants.kPhysicalMaxAngularSpeedRadiansPerSecond / 1000; //10;
    
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI; // / 6; //4;

    public static final double kPXController = .15; // .15; //.5; // 1.5;
    public static final double kPYController = .15; //.15; //.5; // 1.5;
    public static final double kPThetaController = .005; //.15; //= 1;  //= 3; //Power for robot to spin in auto

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
            new TrapezoidProfile.Constraints(
                    kMaxAngularSpeedRadiansPerSecond,
                    kMaxAngularAccelerationRadiansPerSecondSquared);
}

public static final class TrajectoryConstants {

  public static final String New_Path = "New Path";
  public static final String New_Path_Copy = "New Path Copy";

  public static final double MAX_SPEED = 1;
  public static final double MAX_ACCELERATION = 1;
  public static final double DEPLOYED_X_CONTROLLER_P = .015; // .35;
  public static final double DEPLOYED_Y_CONTROLLER_P = .015; //.35;
  public static final double DEPLOYED_THETA_CONTROLLER_P = .08;
}


}

