// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import com.playingwithfusion.CANVenom;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  static CANVenom climber;
  public Climber() {
    climber = new CANVenom(Constants.CLIMBER, MotorType.kBrushed);
  }

  public void climb(double speed) {
    climber.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
