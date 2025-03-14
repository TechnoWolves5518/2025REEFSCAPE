// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Manipulator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoManipulate extends Command {
  /** Creates a new AutoManipulate. */
  Manipulator manipulate;
  boolean stopCheck;
  int timer;
  public AutoManipulate(Manipulator manipulate) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.manipulate = manipulate;
    addRequirements(manipulate);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stopCheck = false;
    timer = 0;
    manipulate.manipulate(Constants.ManipulatorConstants.MANIPULATE_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer < 60) {
      timer++;
    } else {
      stopCheck = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    manipulate.manipulate(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stopCheck;
  }
}
