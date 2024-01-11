// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.Limelight;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.PoseEstimatorSubsystem;



import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.CANifier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;




/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final XboxController xboxController = new XboxController(0);
  public static final CANifier CANIFIER = new CANifier(OperatorConstants.CANIFIER_ID);
  private final SendableChooser<Command> autoChooser;

  public static final LimeLight limeLight  = new LimeLight();
  public static final PoseEstimatorSubsystem poseSubsystem = new PoseEstimatorSubsystem(swerveSubsystem, limeLight);



   // Namee Swerve Driver's buttons
   public static JoystickButton forwardOneMeter;  // Have robot point straight forward ( Away from driverstation)
   public static JoystickButton faceLeft;
   public static JoystickButton faceRight;
   public static JoystickButton resetOdometryButton;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem, xboxController, true));
    
        // Link the Swerve Driver's button names to their number on the controller
        forwardOneMeter = new JoystickButton(xboxController, 1);
        faceLeft = new JoystickButton(xboxController, 5);
        faceRight = new JoystickButton(xboxController, 6);


    // Register Named Commands
     //NamedCommands.registerCommand("New Path", getAutonomousCommand());


    // Configure the trigger bindings
    configureBindings();


  // This Should populate with all Autos (*.auto files) in deploy/pathplanner/autos directory. No need to populate here.
    autoChooser = AutoBuilder.buildAutoChooser("Straight Forward and Back"); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  public static final class Buttons
  {
    public static double getWithDeadZone(double value, double threshold)
    {
      if (Math.abs(value) < threshold) 
      {
          value = 0;
      }
      else
      {
          value = (Math.abs(value) - threshold) * (1.0 / (1 - threshold)) * Math.signum(value);
      }

      return value;
   }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    forwardOneMeter.whileTrue(new InstantCommand(() -> swerveSubsystem.ZeroHeading()));
    faceLeft.whileTrue(new InstantCommand(() -> swerveSubsystem.FaceLeft()));
    faceRight.whileTrue(new InstantCommand(() -> swerveSubsystem.FaceRight()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

   
 public Command getAutonomousCommand() {
 
  // Load the path you want to follow using its name in the GUI
  /////PathPlannerPath path = PathPlannerPath.fromPathFile("New Path");

  // Create a path following command using AutoBuilder. This will also trigger event markers.
    return autoChooser.getSelected();
    //return null;
    //return new PathPlannerAuto("Straight Forward and Back");
    //return new PathPlannerAuto("New Auto");


  }  

}
