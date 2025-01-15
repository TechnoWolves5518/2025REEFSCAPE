// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import com.playingwithfusion.CANVenom;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Manipulator extends SubsystemBase {
  /** Creates a new Manipulator. */
  static CANVenom manipulator;
  public Manipulator() {
    manipulator = new CANVenom(Constants.MANIPULATOR, MotorType.kBrushed);
  }

  public void manipulate(double speed) {
    manipulator.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
