// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.VisionBase;

public class AutoAlign extends SubsystemBase {
  /** Creates a new AutoAlign. */
  /* Define Variables */
  private VisionBase visionSystem;
  public AutoAlign(VisionBase visionSystemVar) {
    // Constructor for the auto align system
    this.visionSystem = visionSystemVar;
  }

  public void AlignRequest() {
    // This method will be called by other functions to request alignment data.
    //TODO implement the request logic
  }

  public void Align() {
    // This method will be called to compute the alignment data.
    //TODO implement the alignment logic
  }

  public void GetX() {
    // This method will be called to retrieve the X offset for auto align.
    //TODO implement the X logic
  }

  public void GetY() {
    // This method will be called to retrieve the Y offset for auto align.
    //TODO implement the Y logic
  }

  public void GetAngle() {
    // This method will be called to retrieve the angle offset for auto align.
    //TODO implement the angle logic
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
