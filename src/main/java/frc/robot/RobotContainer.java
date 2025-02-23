// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

@Logged
public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond); // 2 rotations per second max angular velocity
    private double SlowSpeed = MaxSpeed * 0.25;
    private double SlowAngularRate = MaxAngularRate * 0.25;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Elevator elevator = new Elevator();
    public final Intake intake = new Intake();
    public final Climber climber = new Climber();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            )
        );
        driver.rightBumper().whileTrue(
            drivetrain.applyRequest(() -> 
                drive.withVelocityX(-driver.getLeftY() * SlowSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * SlowSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * SlowAngularRate) // Drive counterclockwise with negative X (left)
                    .withDeadband(SlowSpeed * 0.1).withRotationalDeadband(SlowAngularRate * 0.1) // Add a 10% deadband
            )
        );
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        // reset the field-centric heading on left bumper press
        driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driver.povDown().whileTrue(intake.setPivotForward());
        driver.povUp().whileTrue(intake.setPivotBackward());

        driver.a().whileTrue(intake.setIntakeForward());
        driver.y().whileTrue(intake.setIntakeBackward());

        //driver.b().whileTrue(intake.setHolderForward());
        //driver.x().whileTrue(intake.setHolderBackward());

        driver.rightTrigger().whileTrue(intake.setScorerForward());
        driver.leftTrigger().whileTrue(intake.setScorerBackward());

        operator.povUp().whileTrue(elevator.setElevatorForward());
        operator.povDown().whileTrue(elevator.setElevatorBackward());

        operator.a().whileTrue(elevator.setArmForward());
        operator.y().whileTrue(elevator.setArmBackward());

        operator.b().whileTrue(elevator.setWristForward());
        operator.x().whileTrue(elevator.setWristBackward());

        operator.rightTrigger().whileTrue(climber.setClimberForward());
        operator.leftTrigger().whileTrue(climber.setClimberBackward());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
