// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.driveTrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.driveTrain.SwerveDrive;
import frc.robot.subsystems.driveTrain.Telemetry;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIOTalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  Shooter shooter;
  Indexer indexer;
  Kicker kicker;
  SwerveDrive swerveDrive;
  Hood hood;
  int mode; //

  private ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    this.shooter = new Shooter(new ShooterIOTalonFX());// fixed an error when merging new shooter code
    this.indexer = new Indexer(new IndexerIOTalonFX());
    this.kicker = new Kicker(new KickerIOTalonFX());
    this.swerveDrive = new SwerveDrive();
    this.hood = new Hood(new HoodIOTalonFX());
    this.mode = 0;

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    swerveDrive.drivetrain.setDefaultCommand(swerveDrive.drive(driverController));

        // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driverController
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Reset the field-centric heading on left bumper press.
    driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(logger::telemeterize);

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    driverController.y().whileTrue(shooter.shooterDecrements());
    driverController.x().whileTrue(shooter.shooterIncrements());
    
    // Stop all subsystems (except drivetrain)
    driverController.b().whileTrue(
      Commands.parallel(
        shooter.turnOffShooter(),
        kicker.turnOffKicker(),
        indexer.turnOffIndexer(),
        hood.retractHood()
      )
    );

    // New simpified kicker commands
    driverController.leftTrigger().whileTrue(hood.hoodDecrements());
    driverController.rightTrigger().whileTrue(hood.hoodIncrements());

    driverController.leftBumper().whileTrue(Commands.runOnce(() -> indexer.decrementIndexer()));
    driverController.rightBumper().whileTrue(Commands.runOnce(() -> indexer.incrementIndexer()));

    // Set shooter to velocity 10 and and hood to position 0
    driverController.povLeft().whileTrue(
        Commands.sequence(
            Commands.parallel(
                shooter.setShooterVelocityPosition1().until(shooter.shooterAtVelocitySetPoint()),
                hood.hoodToAnglePosition1().until(hood.hoodAtPositionSetpoint())
            ),
            Commands.parallel(
                kicker.turnOnKicker(),
                indexer.turnOnIndexer()
            )
        )
    );

    // Set shooter to velocity 30 and and hood to position 3
    driverController.povUp().whileTrue(
      Commands.sequence(
            Commands.parallel(
                shooter.setShooterVelocityPosition2().until(shooter.shooterAtVelocitySetPoint()),
                hood.hoodToAnglePosition2().until(hood.hoodAtPositionSetpoint())
            ),
            Commands.parallel(
                kicker.turnOnKicker(),
                indexer.turnOnIndexer()
            )
        )
    );

    // Set shooter to velocity 60 and and hood to position 5
    driverController.povRight().whileTrue(
      Commands.sequence(
            Commands.parallel(
                shooter.setShooterVelocityPosition3().until(shooter.shooterAtVelocitySetPoint()),
                hood.hoodToAnglePosition3().until(hood.hoodAtPositionSetpoint())),
            Commands.parallel(
                kicker.turnOnKicker(),
                indexer.turnOnIndexer()
            )
        )
    );



  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
