// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import org.photonvision.*;
import org.photonvision.targeting.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.*;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private PhotonCamera aprilCam;
  private PhotonTrackedTarget trackedTag;
  private int trackedTagID;
  private PhotonPipelineResult results;
  private boolean targetVisible;
  private double targetYaw;
  private Transform3d targetTransform3d;
  private double targetTranslateX;
  private double targetTranslateY;
  private Transform2d robotToCamera = new Transform2d(Units.inchesToMeters(13.5), Units.inchesToMeters(10.375), new Rotation2d(0));
  private Transform3d robotToCamera3d = new Transform3d(13.5, 10.375, 6, new Rotation3d(0,0,0));
  private Transform2d cameraToRobot = robotToCamera.inverse();

  
  

  public Vision(int TagID) {
    aprilCam = new PhotonCamera("aprilCam");
    if(!aprilCam.isConnected()) {
      DriverStation.reportWarning("AprilTag Camera Missing", false);
    }
    trackedTagID = TagID;
  }

  public void setTrackedTag(int TagID) {
    trackedTagID = TagID;
  }

  public void update() {
    var results = aprilCam.getAllUnreadResults();
    if (!results.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      var result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        // At least one AprilTag was seen by the camera
        for (var target : result.getTargets()) {
          if (target.getFiducialId() == trackedTagID) {
            // Found Tag, record its information
            targetTransform3d = target.getBestCameraToTarget();
            targetTranslateX = targetTransform3d.getX();
            targetTranslateY = targetTransform3d.getY();
            targetYaw = targetTransform3d.getRotation().getZ();
            Pose2d targetPose2d = new Pose2d(targetTranslateX, targetTranslateY, new Rotation2d(targetYaw));
            Pose2d updatedPose2d = targetPose2d.transformBy(cameraToRobot);
            targetYaw = updatedPose2d.getRotation().getRadians();
            targetTranslateX = updatedPose2d.getX();
            targetTranslateY = updatedPose2d.getY();
            targetVisible = true;
          }
          else {
            targetVisible = false;
          }
        }
      }
      else {
        targetVisible = false;
      }
    }
    else {
      if(!aprilCam.isConnected()) {
        targetVisible = false;
        DriverStation.reportError("PhotonVision System Failure - No camera", false);
      }
    }

  }

  public double getYaw() {
    return targetYaw;
  }

  public double getTranslateX() {
    return targetTranslateX;
  }

  public double getTranslateY() {
    return targetTranslateY;
  }

  public void checkConnection() {
    if(!aprilCam.isConnected()) {
      DriverStation.reportWarning("AprilTag Camera 01 Missing", false);
    }

  }
  
  public boolean isVisible() {
    return targetVisible;
  }

  @Override
  public void periodic() {
    this.update();
    this.checkConnection();
    if(this.isVisible()) {
      SmartDashboard.putNumber("Target Yaw", Units.radiansToDegrees(this.getYaw()));
      SmartDashboard.putNumber("Target X", targetTranslateX);
      SmartDashboard.putNumber("Target Y", targetTranslateY);
    }
    SmartDashboard.putBoolean("Target Visible", this.isVisible());
  }
}

