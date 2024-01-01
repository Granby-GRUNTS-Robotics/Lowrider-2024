// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends Command {
  /** Creates a new SwerveJoystickCmd. */
  private final SwerveSubsystem swerveSubsystem;
  private final boolean fieldOrientedFunction;
  XboxController xbox;

  double maxSpeedMPS;
  double maxAngularRPS;
  SlewRateLimiter xLimiter, yLimiter, turnLimiter;

  double red;
  double green;
  double blue;

  boolean rev;

  public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem, XboxController xbox,
  boolean fieldOrientedFunction) {
    // Use addRequirements() here to declare subsystem dependencies.
      
    this.swerveSubsystem = swerveSubsystem;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.xbox = xbox;
    this.xLimiter = new SlewRateLimiter(Constants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(Constants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.turnLimiter = new SlewRateLimiter(Constants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    maxSpeedMPS = Constants.kTeleDriveMaxSpeedMetersPerSecond;
    maxAngularRPS = Constants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    red = 0.0;
    green = 0.5;
    blue = 0.5;
    rev = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = RobotContainer.Buttons.getWithDeadZone(xbox.getLeftX(), 0.1);
    double ySpeed = RobotContainer.Buttons.getWithDeadZone(xbox.getLeftY(), 0.1);
    double directionSpeed = RobotContainer.Buttons.getWithDeadZone(xbox.getRightX(), 0.1); // todd 8-22-23 * maxAngularRPS;

    xSpeed = xLimiter.calculate(xSpeed) * maxSpeedMPS;
    ySpeed = yLimiter.calculate(ySpeed)  * maxSpeedMPS;
    directionSpeed = turnLimiter.calculate(directionSpeed) * maxAngularRPS; // Todd 8-22-23


    ChassisSpeeds chassisSpeeds;
   // if (fieldOrientedFunction)
   // {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, directionSpeed, swerveSubsystem.getRotation2d());  

      SmartDashboard.putNumber("xSpeed", xSpeed);
      SmartDashboard.putNumber("ySpeed", ySpeed);
      SmartDashboard.putNumber("directionSpeed", directionSpeed);
   // }
   // else
   // {
   //   chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, directionSpeed);
   // }

    SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    swerveSubsystem.setModuleStates(moduleStates);

    //LEDs.setLEDColor(Math.abs(xbox.getLeftY()), Math.abs(xbox.getRightX()), Math.abs(xbox.getLeftX()) / 1.5);
    LEDs.setLEDColor(1, 0, 1);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
