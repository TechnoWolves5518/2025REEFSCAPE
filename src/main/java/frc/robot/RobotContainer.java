// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.smartdashboard.*;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants.SwerveConstants;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double TotalMaxSpeed = MaxSpeed * SwerveConstants.speedMultiplier;
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * SwerveConstants.deadband).withRotationalDeadband(MaxAngularRate * SwerveConstants.deadband) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors


    private final SwerveRequest.RobotCentric driveRobot = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * SwerveConstants.deadband).withRotationalDeadband(MaxAngularRate * SwerveConstants.deadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private boolean fieldCentric = false;
    private boolean slowMode = false;

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private double speedMultiplier = 1;
    private double angleMultiplier = 1;
    
    public Command setSlow() {
        return Commands.runOnce(() -> {
            slowMode = !slowMode;
            SmartDashboard.putBoolean("Slow", slowMode);
        });
    }


    public void configDrive() {
        if (slowMode) {
            speedMultiplier = SwerveConstants.SlowSpeed;
            angleMultiplier = SwerveConstants.SlowAngle;
        }
        else {
            speedMultiplier = 1;
            angleMultiplier = 1;
        }
    }


    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * TotalMaxSpeed * speedMultiplier) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * TotalMaxSpeed * speedMultiplier) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * speedMultiplier) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.start().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.back().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));
        
        joystick.leftBumper().onTrue(Commands.runOnce(() -> {slowMode = !slowMode; SmartDashboard.putBoolean("Slow", slowMode);}));
        joystick.rightBumper().whileTrue(drivetrain.applyRequest(() ->
            driveRobot.withVelocityX(-joystick.getLeftY() * TotalMaxSpeed * speedMultiplier)
                .withVelocityY(-joystick.getLeftX() * TotalMaxSpeed * speedMultiplier)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate * speedMultiplier)
        ));

        // reset the field-centric heading on left bumper press
        joystick.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
