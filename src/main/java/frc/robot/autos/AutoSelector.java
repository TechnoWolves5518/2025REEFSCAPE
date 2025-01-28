// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.Manipulator;

/** Add your docs here. */
public class AutoSelector {
    private final SendableChooser<Command> chooser = new SendableChooser<>();


      public AutoSelector(SwerveSubsystem swerve, Manipulator manipulator){
            chooser.setDefaultOption("Default", new PathPlannerAuto("New Auto"));
            chooser.addOption("Tets", new PathPlannerAuto("Test Auto"));

            SmartDashboard.putData(chooser);
      }
      public Command getSelected(){
            return chooser.getSelected();
      }
}
