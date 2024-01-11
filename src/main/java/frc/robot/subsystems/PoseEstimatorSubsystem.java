// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseEstimatorSubsystem extends SubsystemBase {

  private SwerveSubsystem swerveSubsystem;
  private LimeLight limelightSubsystem;

  // stateStdDevs Standard deviations of the pose estimate (x position in meters, y position in meters, and heading in radians). 
  // Increase these numbers to trust your state estimate less.
  private static final edu.wpi.first.math.Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05,
      Units.degreesToRadians(5));

   /**  How much should we trust what the camera is seeing vrs where our encoders think we are at
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   */
  private static final edu.wpi.first.math.Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5,
      Units.degreesToRadians(5));

  private final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d field2d = new Field2d();

  private double previousPipelineTimestamp = 0;


  public PoseEstimatorSubsystem(SwerveSubsystem swervesubsystem, LimeLight limelight) {
    this.swerveSubsystem = swervesubsystem;
    this.limelightSubsystem = limelight;

    ShuffleboardTab tab = Shuffleboard.getTab("Vision");

    poseEstimator = new SwerveDrivePoseEstimator(
        swerveSubsystem.getKinematics(),
        swerveSubsystem.getRotation2d(),
        swerveSubsystem.getModulePosition(),
        new Pose2d(),
        stateStdDevs,
        visionMeasurementStdDevs);

    tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
    tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
  }

  @Override
  public void periodic() {
    //System.out.println(driveSubsystem.getPose());
    var resultTimestamp = limelightSubsystem.getLatency();
    if(limelightSubsystem.getTargets() && limelightSubsystem.getArea() > 0.120 && resultTimestamp != previousPipelineTimestamp) {

      previousPipelineTimestamp = resultTimestamp;  // Make current image from limight the one we are processing so we won't process it again

      double[] botPose = limelightSubsystem.getBotPoseBlue();  // get position of robot on field based on BLue Alliance starting pose corner
      
      //Breakout the values returned in botPose above into its parts and pass to get location of robot
      // 0 = x location, 1 = Y location, 2 = z location, 3 = Roll, 4 = Pitch, 5 = Yaw, 6 = total latency (cl+tl)
      Pose3d actualPose = new Pose3d(botPose[0], botPose[1], botPose[2], new Rotation3d(Units.degreesToRadians(botPose[3]), Units.degreesToRadians(botPose[4]), Units.degreesToRadians(botPose[5])));
      
      //System.out.println(Timer.getFPGATimestamp() - (botPose[6]/1000.0));
      //Pose3d camPose = new Pose3d(new Translation3d(5, new Rotation3d(visionArray[3], visionArray[4], visionArray[5])), new Rotation3d());
      
      /*Transform3d camToTarget = target.getBestCameraToTarget();
      Pose3d camPose = targetPose.transformBy(camToTarget.inverse());*/

      poseEstimator.addVisionMeasurement(actualPose.toPose2d(), Timer.getFPGATimestamp() - (botPose[6]/1000.0));
    }

    poseEstimator.update(
      swerveSubsystem.getRotation2d(),
      swerveSubsystem.getModulePosition());

    field2d.setRobotPose(getCurrentPose());
  }

  private String getFomattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f) %.2f degrees", 
        pose.getX(), 
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose() {
    poseEstimator.resetPosition(swerveSubsystem.getRotation2d(),
    swerveSubsystem.getModulePosition(), getCurrentPose());
  }
}