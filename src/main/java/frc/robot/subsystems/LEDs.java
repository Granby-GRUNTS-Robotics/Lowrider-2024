// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.w3c.dom.css.RGBColor;

import com.ctre.phoenix.CANifier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.OperatorConstants;

public class LEDs extends SubsystemBase {
  /** Creates a new LEDs. */
  private final static CANifier mCanifierLights = RobotContainer.CANIFIER;
  public LEDs() {}

  @Override 
  public void periodic() {
    // This method will be called once per scheduler run loop
  }

  public static void setLEDColor(double red, double green, double blue) {
    mCanifierLights.setLEDOutput(green, CANifier.LEDChannel.LEDChannelA); //green A
    mCanifierLights.setLEDOutput(red, CANifier.LEDChannel.LEDChannelB); //red  B
    mCanifierLights.setLEDOutput(blue, CANifier.LEDChannel.LEDChannelC); //blue  C
  }

}
