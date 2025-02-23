// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Manipulator extends SubsystemBase {
  /** Creates a new Manipulator. */
  public static SparkMax manipulatorLeft;
  public static SparkMax manipulatorRight;

  public Manipulator() {
    manipulatorLeft = new SparkMax(Constants.MANIPULATORLEFT, SparkMax.MotorType.kBrushless);
    manipulatorRight = new SparkMax(Constants.MANIPULATORRIGHT, SparkMax.MotorType.kBrushless);
  }

  public void manipulate(double speed) {
    manipulatorLeft.set(-speed);
    manipulatorRight.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
