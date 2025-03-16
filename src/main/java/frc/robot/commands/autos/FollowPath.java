// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FollowPath extends Command {
  /** Creates a new FollowPath. */
  String pathName;
  int timer;
  int time;
  boolean stopCheck;
  public FollowPath(String pathName, int time) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pathName = pathName;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
    stopCheck = false;
    CommandSwerveDrivetrain.followPath(pathName);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer < time) {
      timer++;
    } else {
      stopCheck = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stopCheck;
  }
}
