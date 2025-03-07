// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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
        configureBindings();
        configureNamedCommands();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed * 0.8) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed * 0.8) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate * 0.8) // Drive counterclockwise with negative X (left)
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
        //driver.back().and(driver.a()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //driver.start().and(driver.a()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        // reset the field-centric heading on left bumper press
        driver.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //driver.povLeft().whileTrue(drivetrain.driveToReefPoint(true));
        //driver.povRight().whileTrue(drivetrain.driveToReefPoint(false));
        driver.leftTrigger().whileTrue(drivetrain.driveToReefPoint(true));
        driver.rightTrigger().whileTrue(drivetrain.driveToReefPoint(false));
        //driver.povUp().whileTrue(drivetrain.driveToCoralStationPoint(true));
        //driver.povDown().whileTrue(drivetrain.driveToCoralStationPoint(false));
        driver.b().whileTrue(drivetrain.driveToCoralStationPoint(true));
        driver.a().whileTrue(drivetrain.driveToCoralStationPoint(false));

        //driver.leftBumper().whileTrue(intake.intakeAlgae());
        driver.leftBumper().whileTrue(Commands.either(
            intake.setScorerBackward(),
            intake.setScorerBackwardSlow(),
            () -> elevator.elevatorPosition() > 2.0));

        //driver.a().whileTrue(intake.scoreAlgae());

        //driver.rightTrigger().whileTrue(intake.setScorerForward());
        //driver.leftTrigger().whileTrue(intake.setScorerBackward());

        driver.x().whileTrue(intake.scoreAlgae());
        driver.y().whileTrue(intake.setScorerForward());

        operator.leftBumper().onTrue(elevator.moveToCoralStation());
        operator.rightBumper().whileTrue(intake.intakeAlgae());

        operator.a().onTrue(elevator.moveToL1());
        operator.b().onTrue(elevator.moveToL2());
        operator.x().onTrue(elevator.moveToL3());
        operator.y().onTrue(elevator.moveToL4());

        operator.povDown().onTrue(Commands.sequence(
            elevator.moveToAlgaeLow(),
            intake.scorerForward()));
        operator.povUp().onTrue(Commands.sequence(
            elevator.moveToAlgaeHigh(),
            intake.scorerForward()));

        operator.start().onTrue(elevator.moveElevatorUp());
        operator.back().onTrue(elevator.moveElevatorDown());

        operator.rightTrigger().whileTrue(climber.setClimberForward());
        operator.leftTrigger().whileTrue(climber.setClimberBackward());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void configureNamedCommands() {
        NamedCommands.registerCommand("ElevatorToCoralStation", Commands.sequence(
            Commands.waitSeconds(0.5),
            elevator.moveToCoralStation()));
        NamedCommands.registerCommand("ElevatorToL4", elevator.moveToL4());
        NamedCommands.registerCommand("ScoreCoral", Commands.sequence(
            Commands.waitSeconds(0.5),
            intake.setScorerBackward().withTimeout(0.5)
        ));
        NamedCommands.registerCommand("IntakeCoral", Commands.sequence(
            intake.setScorerBackwardVoltage().until(() -> intake.hasCoral()),
            intake.setScorerBackward().withTimeout(0.1)
        ));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    public Command resetScorer() {
        return intake.setScorerPosition();
    }
}
