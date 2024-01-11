// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
  import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
  import edu.wpi.first.math.kinematics.SwerveModulePosition;
  import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.util.PathPlannerLogging;



public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */

  SwerveModulePosition[] moduleStates;
  private final SwerveModule frontLeft = new SwerveModule(
    Constants.OperatorConstants.LEFT_FRONT_SPEED_MOTOR_ID,      //can ID for speed
    Constants.OperatorConstants.LEFT_FRONT_DIRECTION_MOTOR_ID,  //can ID for direction
    Constants.LEFT_FRONT_SPEED_IS_REVERSED,                     //boolean for if the speed motor is set to reverse 
    Constants.LEFT_FRONT_DIRECTION_IS_REVERSED,                 //boolean for if the direction motor is set to reverse
    Constants.OperatorConstants.LEFT_FRONT_CANCODER_ID,         //can ID for cancoder
    Constants.LEFT_FRONT_RADIAN_OFFSET,                         //to correct for encoder, so 0 is actually pointing ahead
    Constants.LEFT_FRONT_CANCODER_IS_REVERSED
  );

  private final SwerveModule frontRight = new SwerveModule(
    Constants.OperatorConstants.RIGHT_FRONT_SPEED_MOTOR_ID,      //can ID for speed
    Constants.OperatorConstants.RIGHT_FRONT_DIRECTION_MOTOR_ID,  //can ID for direction
    Constants.RIGHT_FRONT_SPEED_IS_REVERSED,                     //boolean for if the speed motor is set to reverse 
    Constants.RIGHT_FRONT_DIRECTION_IS_REVERSED,                 //boolean for if the direction motor is set to reverse
    Constants.OperatorConstants.RIGHT_FRONT_CANCODER_ID,         //can ID for cancoder
    Constants.RIGHT_FRONT_RADIAN_OFFSET,                         //to correct for encoder, so 0 is actually pointing ahead
    Constants.RIGHT_FRONT_CANCODER_IS_REVERSED
  );

  private final SwerveModule backLeft = new SwerveModule(
    Constants.OperatorConstants.LEFT_BACK_SPEED_MOTOR_ID,      //can ID for speed
    Constants.OperatorConstants.LEFT_BACK_DIRECTION_MOTOR_ID,  //can ID for direction
    Constants.LEFT_BACK_SPEED_IS_REVERSED,                     //boolean for if the speed motor is set to reverse 
    Constants.LEFT_BACK_DIRECTION_IS_REVERSED,                 //boolean for if the direction motor is set to reverse
    Constants.OperatorConstants.LEFT_BACK_CANCODER_ID,         //can ID for cancoder
    Constants.LEFT_BACK_RADIAN_OFFSET,                         //to correct for encoder, so 0 is actually pointing ahead
    Constants.LEFT_BACK_CANCODER_IS_REVERSED
  );

  private final SwerveModule backRight = new SwerveModule(
    Constants.OperatorConstants.RIGHT_BACK_SPEED_MOTOR_ID,      //can ID for speed
    Constants.OperatorConstants.RIGHT_BACK_DIRECTIION_MOTOR_ID,  //can ID for direction
    Constants.RIGHT_BACK_SPEED_IS_REVERSED,                     //boolean for if the speed motor is set to reverse 
    Constants.RIGHT_BACK_DIRECTION_IS_REVERSED,                 //boolean for if the direction motor is set to reverse
    Constants.OperatorConstants.RIGHT_BACK_CANCODER_ID,         //can ID for cancoder
    Constants.RIGHT_BACK_RADIAN_OFFSET,                         //to correct for encoder, so 0 is actually pointing ahead
    Constants.RIGHT_BACK_CANCODER_IS_REVERSED
  );

  private final Pigeon2 gyro = new Pigeon2(Constants.PIGEON_ID);

   //Create Odometer for swerve drive
  private SwerveDriveOdometry odometer;

  //////////// Trying path planner 12-16-23
  private SwerveDriveKinematics kinematics;  // Trying Pathplanner




  // Return position of the swerve module for odometry
  public SwerveModulePosition[] getModulePosition(){
    return (new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
      });
    }

    // Return State of the swerve module for odometry 
  public SwerveModuleState[] getModuleStates(){
    return (new SwerveModuleState[] {
      frontLeft.getState(),
      frontRight.getState(),
      backLeft.getState(),
      backRight.getState()
      });
      
  }
/////////////////////////////////////////////////

 

  //private final SwerveDrivePoseEstimator odometry;  // Odemeter for when we have vision to help drive the robot


  // Swerve subsystem constructor 
  public SwerveSubsystem() {
   
  //Restart robot encoders on startup
  resetAllEncoders();

  gyro.configFactoryDefault();
  zeroGyro();

   kinematics = new SwerveDriveKinematics(
    new Translation2d(Constants.kWheelBase / 2, Constants.kTrackWidth / 2),     // Front Left wheel position from center of robot
    new Translation2d(Constants.kWheelBase / 2, -Constants.kTrackWidth / 2),    // Front Right wheel postion
    new Translation2d(-Constants.kWheelBase / 2, Constants.kTrackWidth / 2),    // Back Left wheel postion
    new Translation2d(-Constants.kWheelBase / 2, -Constants.kTrackWidth / 2)    // Back Right wheel position
  );



   // kinematics = new SwerveDriveKinematics(
   // new Translation2d(Constants.kWheelBase / 2, -Constants.kTrackWidth / 2),     // Front Right wheel position from center of robot
   // new Translation2d(Constants.kWheelBase / 2, Constants.kTrackWidth / 2),    // Front Left wheel postion
   // new Translation2d(-Constants.kWheelBase / 2, -Constants.kTrackWidth / 2),    // Back Right wheel postion
   // new Translation2d(-Constants.kWheelBase / 2, Constants.kTrackWidth / 2)    // Back Left wheel position
  //);

  odometer = new SwerveDriveOdometry(kinematics,
    getRotation2d(), 
    getModulePosition()); 

    //odometer = new SwerveDriveOdometry(kinematics,
    //getHeadingRot2d(), 
    //getModulePosition(),
    //new Pose2d()); 

    //////////// Trying path planner 12-16-23
 // kinematics = new SwerveDriveKinematics(
 //     Constants.Swerve.flModuleOffset, 
  //    Constants.Swerve.frModuleOffset, 
  //    Constants.Swerve.blModuleOffset, 
  //    Constants.Swerve.brModuleOffset
  //  );



  // Configure AutoBuilder
  AutoBuilder.configureHolonomic(
    this::getPose, 
    this::resetOdometry, 
    this::getSpeeds, 
    this::driveRobotRelative, 
    Constants.Swerve.pathFollowerConfig, 
    () -> {
                    // For 2024 Game: Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
    this
  ); 

  
////////////////////////
  }

  public double getHeading()
  {
    //SmartDashboard.putNumber("Compass Heading", gyro.getAbsoluteCompassHeading());
    return Math.IEEEremainder(gyro.getYaw(), 360);
    //return gyro.getYaw();
  }

  public Rotation2d  getHeadingRot2d()
  {
    return Rotation2d.fromDegrees(gyro.getYaw());
  }

  public Rotation2d getRotation2d()
  {
    return Rotation2d.fromDegrees(getHeading());
  }

    // Get the location determined by the odometer.  Returns x and y location in meters
  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  // Reset the odometer to a new location  **Note Pose2d contains transation2d (X, y coordinates on field and Rotation2d which is where robot is facing)
public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(
      getRotation2d(),
      getModulePosition(), 
      pose);
}


public void resetAllEncoders()
{
  frontLeft.resetEncoders();
  frontRight.resetEncoders();
  backLeft.resetEncoders();
  backRight.resetEncoders();
 }

  public double  getRoll()
  {
    return (gyro.getRoll());
  }

  public void zeroGyro() {
    gyro.setYaw(0);
    // gyroOffset = (DriverStation.getAlliance() == Alliance.Blue ? 0 : 180) % 360;
    //gyro.setYaw(90);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("just yaw", gyro.getYaw());
    SmartDashboard.putNumber("Roll", gyro.getRoll());
    SmartDashboard.putNumber("Robot Yaw", getHeading());
    //SmartDashboard.putNumber("Compass Heading", gyro.getAbsoluteCompassHeading());
    SmartDashboard.putNumber("Left Front Encoder Rad", frontLeft.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("Right Front Encoder Rad", frontRight.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("Left Back Encoder Rad", backLeft.getAbsoluteEncoderRad());
    SmartDashboard.putNumber("Right Back Encoder Rad", backRight.getAbsoluteEncoderRad());
    
    SmartDashboard.putNumber("Left Front Angle", frontLeft.getAbsoluteEncoderAngle());
    SmartDashboard.putNumber("Right Front Angle", frontRight.getAbsoluteEncoderAngle());
    SmartDashboard.putNumber("Left Back Angle", backLeft.getAbsoluteEncoderAngle());
    SmartDashboard.putNumber("Right Back Angle", backRight.getAbsoluteEncoderAngle());

    SmartDashboard.putNumber("Left front distance in Meters", frontLeft.getSpeedPosition());
    SmartDashboard.putNumber("Right front distance in Meters", frontRight.getSpeedPosition());
    SmartDashboard.putNumber("Left back distance in Meters", backLeft.getSpeedPosition());
    SmartDashboard.putNumber("Right back distance in Meters", backRight.getSpeedPosition());

    SmartDashboard.putNumber("Left front direction position", frontLeft.getDirectionPosition());



    odometer.update(getRotation2d(), 
                    getModulePosition()
                  );
  }

  public void addPoseEstimatorSwerveMeasurement() {
    odometer.update(
      getRotation2d(),
      getModulePosition()
    );
  }

  public void stopModules()
  {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates)
  {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.physicalMaxSpeedMPS); //NormalizeWheelSpeeds doesn't exist, this is the closest thing idk
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

    public SwerveDriveKinematics getKinematics() {
    return kinematics;
   }
 
 //////////// Trying path planner 12-16-23
  public ChassisSpeeds getSpeeds() {
    //return kinematics.toChassisSpeeds(getModuleStates());
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    //ChassisSpeeds targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(robotRelativeSpeeds, getRotation2d());
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, .02);

    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }
////////////////////////////////////////

//// Button commands for swerve drive

 // This example will simply move the robot 2m forward of its current position
public void ZeroHeading(){
      Pose2d currentPose = getPose();
      
      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
      Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(1.0, 0.0)), new Rotation2d());

      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
      PathPlannerPath path = new PathPlannerPath(
        bezierPoints, 
        new PathConstraints(
          1.0, 1.0, 
          Units.degreesToRadians(360), Units.degreesToRadians(540)
        ),  
        new GoalEndState(0.0, currentPose.getRotation())
      );

      AutoBuilder.followPath(path).schedule();
    }

public void FaceLeft(){
      Pose2d currentPose = getPose();
      
      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
      Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(0.1, 0.0)), new Rotation2d());

      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
      PathPlannerPath path = new PathPlannerPath(
        bezierPoints, 
        new PathConstraints(
          1.0, 1.0, 
          Units.degreesToRadians(360), Units.degreesToRadians(540)
        ),  
        new GoalEndState(0.0, new Rotation2d().plus(Rotation2d.fromDegrees(90)))
      );

      AutoBuilder.followPath(path).schedule();
    }

public void FaceRight(){
      Pose2d currentPose = getPose();
      
      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
      Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(0.1, 0.0)), new Rotation2d());

      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
      PathPlannerPath path = new PathPlannerPath(
        bezierPoints, 
        new PathConstraints(
          1.0, 1.0, 
          Units.degreesToRadians(360), Units.degreesToRadians(540)
        ),  
        new GoalEndState(0.0, new Rotation2d().plus(Rotation2d.fromDegrees(270)))
      );

      AutoBuilder.followPath(path).schedule();
    }

    public void PathFind(){
    AutoBuilder.pathfindToPose(
      new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)), 
      new PathConstraints(
        1.0, 1.0, 
        Units.degreesToRadians(360), Units.degreesToRadians(540)
      ), 
      0, 
      2.0
    );
    }
    

}
