// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;

@Logged
public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond); // 2 rotations per second max angular velocity
    private double NormalSpeed = MaxSpeed * 0.75; // Normal drive speed is 75% of max speed
    private double NormalAngularRate = MaxAngularRate * 0.75; // Normal rotation rate is 75% of max rotation rate
    private double SlowSpeed = MaxSpeed * 0.25; // Slow drive speed is 25% of max speed
    private double SlowAngularRate = MaxAngularRate * 0.225; // Slow rotation rate is 22.5% of max rotation rate

    // Drive settings
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Controllers
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Elevator elevator = new Elevator();
    public final Intake intake = new Intake();
    public final Climber climber = new Climber();
    public final LED led = new LED();
    

    public final Trigger coralSensor;

    // Autonomous selector
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        coralSensor = new Trigger(() -> intake.hasCoral());
        

        configureBindings();
        configureNamedCommands();

    
        // Place Autonomous selector on SmartDashboard
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * NormalSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * NormalSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * NormalAngularRate) // Drive counterclockwise with negative X (left)
                    .withDeadband(NormalSpeed * 0.1).withRotationalDeadband(NormalAngularRate * 0.1) // Add a 10% deadband
            )
        );

        // Drive slowly when driver right bumper is held
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

        // Reset the field-centric heading on driver start press
        driver.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Drive to left/right reef points on driver left/right trigger press
        driver.leftTrigger().whileTrue(drivetrain.driveToReefPoint(true));
        driver.rightTrigger().whileTrue(drivetrain.driveToReefPoint(false));

        // Intake and score coral on driver left bumper press
        driver.leftBumper().whileTrue(Commands.either(
            Commands.either(
                intake.scoreCoralL3(),
                intake.handleCoral(),
                () -> elevator.elevatorPosition() > 22.0 && elevator.elevatorPosition() < 25.0),
            intake.scoreCoralL1(),
            () -> elevator.elevatorPosition() > 4.0 || elevator.armPosition() < 0.0
        ));

        coralSensor.onTrue(Commands.runOnce(()-> {
          led.purplelight();
        }).ignoringDisable(true));

        coralSensor.onFalse(Commands.runOnce(()-> {
            led.rainbowlight();
          }).ignoringDisable(true));

        // Move to coral station automatically when driver left bumper is released at L3 or L4
        driver.leftBumper().onFalse(Commands.either(
            elevator.moveToCoralStation(),
            Commands.none(),
            () -> elevator.elevatorPosition() > 20.0
        ));

        // Eject coral on driver y press
        driver.y().whileTrue(intake.scoreCoralL1());

        driver.b().onTrue(climber.moveClimberOut());
        driver.a().onTrue(climber.moveClimberIn());
        driver.x().onTrue(climber.moveClimberReset());

        // Move to coral station on operator left bumper press
        operator.leftBumper().onTrue(elevator.moveToCoralStation());

        // Move to floor pickup on operator right bumper press
        operator.rightBumper().onTrue(Commands.sequence(
            elevator.moveToFloorPickup(),
            intake.runScorer()));

        // Move to L1-L4 on operator a/b/x/y press
        operator.a().onTrue(elevator.moveToL1());
        operator.b().onTrue(elevator.moveToL2());
        operator.x().onTrue(elevator.moveToL3());
        operator.y().onTrue(elevator.moveToL4());

        // Move to algae low/high on operator pov down/up press
        operator.povDown().onTrue(Commands.sequence(
            elevator.moveToAlgaeLow(),
            intake.removeAlgae()));
        operator.povUp().onTrue(Commands.sequence(
            elevator.moveToAlgaeHigh(),
            intake.removeAlgae()));
        operator.povRight().onTrue(Commands.sequence(
            elevator.moveToAlgaeScoreLow(),
            intake.scoreCoralL1().withTimeout(0.03),
            Commands.runOnce(()->led.greenlight())));
        operator.povLeft().onTrue(Commands.sequence(
            elevator.moveToAlgaeScoreHigh(),
            intake.scoreCoralL1().withTimeout(0.03),
            Commands.runOnce(()->led.greenlight())));

        // Bump elevator up/down on operator start/back press
        operator.start().onTrue(elevator.moveElevatorUp());
        operator.back().onTrue(elevator.moveElevatorDown());

        // Move the climber out/in on operator left/right trigger press
        operator.rightTrigger().whileTrue(climber.setClimberForward());
        operator.leftTrigger().whileTrue(climber.setClimberBackward());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void configureNamedCommands() {
        NamedCommands.registerCommand("ElevatorToCoralStation", Commands.sequence(
            Commands.waitSeconds(0.5),
            elevator.moveToCoralStation()));
        NamedCommands.registerCommand("ElevatorToL4", elevator.moveToL4());
        NamedCommands.registerCommand("ElevatorToL2", elevator.moveToL2());
        NamedCommands.registerCommand("ScoreCoral", Commands.sequence(
            Commands.waitSeconds(0.5),
            intake.handleCoral().withTimeout(0.5)
        ));
        NamedCommands.registerCommand("ScoreCoralFast", Commands.sequence(
            Commands.waitSeconds(0.3),
            intake.handleCoral().withTimeout(0.2)
        ));
        NamedCommands.registerCommand("IntakeCoral", Commands.sequence(
            intake.handleCoralWithSensor(),
            intake.handleCoral().withTimeout(0.1)
        ));
        NamedCommands.registerCommand("IntakeCoralGround", Commands.sequence(
            elevator.moveToFloorPickup(),
            intake.handleCoral()
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
