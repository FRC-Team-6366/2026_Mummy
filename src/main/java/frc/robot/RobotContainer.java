// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.driveTrain.Drive;
import frc.robot.subsystems.driveTrain.DriveCommands;
import frc.robot.subsystems.driveTrain.GyroIO;
import frc.robot.subsystems.driveTrain.GyroIOPigeon2;
import frc.robot.subsystems.driveTrain.ModuleIO;
import frc.robot.subsystems.driveTrain.ModuleIOSim;
import frc.robot.subsystems.driveTrain.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerIOSim;
import frc.robot.subsystems.kicker.KickerIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.shooter.hood.HoodIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  public Shooter shooter;
  public Indexer indexer;
  public Kicker kicker;
  public Hood hood;
  public int mode; 
  public Drive drive;
  public Intake intake;
  public Vision vision;
  LoggedDashboardChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(
      OperatorConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive = new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight));

        vision = new Vision(drive::addVisionMeasurement,
            new VisionIOPhotonVision(Constants.VisionConstants.camera0Name,
                Constants.VisionConstants.robotToCamera0),
            new VisionIOPhotonVision(Constants.VisionConstants.camera1Name,
                Constants.VisionConstants.robotToCamera1),
            new VisionIOPhotonVision(Constants.VisionConstants.camera2Name,
                Constants.VisionConstants.robotToCamera2),
            new VisionIOPhotonVision(Constants.VisionConstants.camera3Name,
                Constants.VisionConstants.robotToCamera3));

        this.shooter = new Shooter(new ShooterIOTalonFX());// fixed an error when merging new shooter code
        this.hood = new Hood(new HoodIOTalonFX());
        this.indexer = new Indexer(new IndexerIOTalonFX());
        this.intake = new Intake(new IntakeIOTalonFX());
        this.kicker = new Kicker(new KickerIOTalonFX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(
            new GyroIO() {},
            new ModuleIOSim(TunerConstants.FrontLeft),
            new ModuleIOSim(TunerConstants.FrontRight),
            new ModuleIOSim(TunerConstants.BackLeft),
            new ModuleIOSim(TunerConstants.BackRight));
        this.vision = new Vision(
            drive::addVisionMeasurement,
            new VisionIOPhotonVisionSim(
                Constants.VisionConstants.camera0Name,
                Constants.VisionConstants.robotToCamera0, drive::getPose),
            new VisionIOPhotonVisionSim(
                Constants.VisionConstants.camera1Name,
                Constants.VisionConstants.robotToCamera1, drive::getPose),
            new VisionIOPhotonVisionSim(
                Constants.VisionConstants.camera2Name,
                Constants.VisionConstants.robotToCamera2, drive::getPose),
            new VisionIOPhotonVisionSim(
                Constants.VisionConstants.camera3Name,
                Constants.VisionConstants.robotToCamera3, drive::getPose));
        this.shooter = new Shooter(new ShooterIOSim());
        this.hood = new Hood(new HoodIOSim());
        this.indexer = new Indexer(new IndexerIOSim());
        this.intake = new Intake(new IntakeIOSim());
        this.kicker = new Kicker(new KickerIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
        break;
    }

    this.mode = 0;

    // Naming commands for Autos
    NamedCommands.registerCommand("StopWithX", Commands.run(drive::stopWithX, drive));
    NamedCommands.registerCommand("IntakeRunRollers", intake.intakeRunRollers());
    NamedCommands.registerCommand("IntakeStopRollers", intake.intakeStopRollers());
    NamedCommands.registerCommand("ShooterSpinUp", autoShooterSpinUp());
    NamedCommands.registerCommand("AutoShooterSixSeconds", autoShootForSixSeconds());
    NamedCommands.registerCommand("AutoShooterEndless", autoShootForever());
    NamedCommands.registerCommand("Stop", autoTurnOffAllButIntake());
    NamedCommands.registerCommand("IntakeDeploy",
        intake.deployIntake().until(intake.intakePivotAtPositionSetpoint()));
    NamedCommands.registerCommand("IntakeRetract",
        intake.retractIntake().until(intake.intakePivotAtPositionSetpoint()));
    NamedCommands.registerCommand("IntakeHalfRetract",
        intake.intakePivotToAngle(Constants.IntakeConstants.intakePivotPulseUpAngleDegrees));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

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

    // |==============================|
    // | Driver Controls |
    // |==============================|

    // Set the drivers movement for steering and driving on the driver joysticks
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Lock to 0° when A button is held
    driverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> Rotation2d.kZero));

    // Lock to Hub when RT is held
    driverController.leftTrigger().whileTrue(
        DriveCommands.joystickDriveAutoAim(
            drive,
            () -> -driverController.getLeftY(), () -> -driverController.getLeftX()));

    // Run intake rollers when RT is pressed
    driverController.rightTrigger().whileTrue(intake.intakeRunRollers());

    // Switch to X pattern when X button is pressed
    driverController.x().onTrue( Commands.run(drive::stopWithX, drive));
    // driverController.x().onTrue(NamedCommands.getCommand("indexerPulse"));

    driverController.rightBumper().onTrue(this.intake.toggleIntakePivot());

    // |==============================|
    // | Operator Controls |
    // |==============================|

    // Run intake rollers when LT is pressed
    operatorController.leftTrigger().whileTrue(intake.intakeRunRollers());

    operatorController.rightBumper().whileTrue(passFuel());

    // Auto speed and angle when RT is held
    operatorController.rightTrigger().whileTrue(this.autoShootForever());

    // Stop all subsystems (except drivetrain)
    operatorController.b().whileTrue(turnOffAll());

    // Resets cancoder to 0.12 rotations when start and back are pressed together,
    // must be used when intake is at upper hard limit
    operatorController.start().and(operatorController.back()).onTrue(intake.intakeResetCanCoder());

    // Shooting setpoints
    operatorController.a().whileTrue(shootAtPostion1());

    operatorController.x().whileTrue(intake.intakePulsePivot());

    // operatorController.y().whileTrue(this.miataWink());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /**
   * Sets the rumble for both the driver and operator controllers
   * @param power 0.0 to 1.0: 0.0 being off, 1.0 being full power rumble on the controller
   */
  public void rumbleBoth(double power) {
    driverController.setRumble(GenericHID.RumbleType.kBothRumble, power);
    operatorController.setRumble(GenericHID.RumbleType.kBothRumble, power);
  }

  public void rumblePulseBoth(double power){
    //no rumble yet
  }

  // |==============================|
  // | Auto Commands |
  // |==============================|

  /**
   * Returns command for running the shooter, hood, kicker and indexer
   * for 6 seconds to support autonomous mode
   * 
   * @return Command for auto-shooting for 6 seconds (Excluding the drive train)
   */
  public Command autoShootForSixSeconds() {
    return Commands.race(
        Commands.parallel(
            shooter.setShooterAutoVelocity(drive),
            hood.setHoodAutoAngle(drive),
            indexer.pulseIndexer(),
            kicker.runKicker()),
        new WaitCommand(6.0)).withName("autoShootForSixSeconds");
  }

  /**
   * Returns command to spin up the shooter to shooting velocity as well as move the hood to its position
   * based on distance from the goal
   * 
   * @return Command to spin up shooter and move hood to angle for shooting
   */
  public Command autoShooterSpinUp() {
    return Commands.parallel(
        shooter.setShooterAutoVelocity(drive).until(shooter.shooterAtVelocitySetPoint()),
        hood.setHoodAutoAngle(drive).until(hood.hoodAtPositionSetpoint())).withName("autoShooterSpinUp");
  }

  /**
   * Returns command to run shooter and hood based on distance to goal as 
   * well as run the indexer (either pulse or full)
   * 
   * @return Command for auto-shooting
   */
  public Command autoShootForever() {
    return Commands.parallel(
        shooter.setShooterAutoVelocity(drive),
        hood.setHoodAutoAngle(drive),
        Commands.repeatingSequence(Commands.race(
            indexer.runIndexer(),
            kicker.runKicker())))
        .withName("autoShootForever");
  }

  /**
   * Command to turn off all subsystems except for Intake, Vision and Drive
   * @return Command to turn off most subsystems
   */
  public Command autoTurnOffAllButIntake() {
    return Commands.parallel(
        shooter.shooterTurnOff(),
        hood.retractHood(),
        indexer.stopIndexer(),
        kicker.stopKicker()).withName("autoTurnOffAllButIntake");
  }

  // |==============================|
  // | TeleOp Commands |
  // |==============================|

  /**
   * Returns command to turn off shooter, kicker, indexer, hood and intake roller
   * subsystems
   * @return Command to turn off subsystems
   */
  public Command turnOffAll() {
    return Commands.parallel(
        shooter.shooterTurnOff(),
        kicker.stopKicker(),
        indexer.stopIndexer(),
        hood.retractHood(),
        intake.intakeStopRollers()).withName("turnOffAll");
  }

  /**
   * Command to set shooter velocity and hood position to shoot
   * from in front of the tower
   * @return Command to run shooter and hood for shooting from the tower
   */
  public Command shootAtPostion1() {
    return Commands.sequence(
        Commands.parallel(
            shooter.setShooterVelocityPosition1().until(
                shooter.shooterAtVelocitySetPoint()),
            hood.hoodToAnglePosition1()
                .until(hood.hoodAtPositionSetpoint())
        ),
        Commands.parallel(
            shooter.setShooterVelocityPosition1(),
            hood.hoodToAnglePosition1(),
            kicker.runKicker(),
            indexer.pulseIndexer()
        )
    ).withName("shootAtPostion1");
  }

  /**
   * Returns command for shooting fuel from the middle area of the 
   * field to the alliance side of the field
   * @return Command to set shooter velocity, hood position, kicker and to run indexer
   */
  public Command passFuel() {
    return Commands.parallel(
        shooter.setShooterVelocityPosition3(),
        hood.hoodsToAngle(45),
        kicker.runKicker(),
        Commands.repeatingSequence(Commands.race(
            indexer.runIndexer(),
            new WaitCommand(0.5)),
            Commands.race(
                indexer.stopIndexer(),
                new WaitCommand(0.2))))
        .withName("passFuel");
  }

  // |==============================|
  // | Meme Commands |
  // |==============================|

  // public Command miataWink() {
  // return Commands.repeatingSequence(
  // Commands.parallel(
  // hood.hoodToAngleLeft(0).until(hood.hoodAtPositionSetpoint()),
  // hood.hoodToAngleRight(20).until(hood.hoodAtPositionSetpoint())
  // ),
  // Commands.parallel(
  // hood.hoodToAngleLeft(20).until(hood.hoodAtPositionSetpoint()),
  // hood.hoodToAngleRight(0).until(hood.hoodAtPositionSetpoint())
  // )
  // );
  // }
}
