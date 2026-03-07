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
import frc.robot.subsystems.shooter.ShooterIOTalonFXSim;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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
    Hood hood;
    int mode; //
    Drive drive;
    Intake intake;
    Vision vision;
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
                        new GyroIO() {
                        },
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
                        Constants.VisionConstants.robotToCamera3, drive::getPose)
                    );
                this.shooter = new Shooter(new ShooterIOTalonFXSim());
                this.hood = new Hood(new HoodIOSim());
                this.indexer = new Indexer(new IndexerIOSim());
                this.intake = new Intake(new IntakeIOSim());
                this.kicker = new Kicker(new KickerIOSim());
                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        });
                break;
        }

        
        this.mode = 0;

                NamedCommands.registerCommand("IntakeDeploy",
                                intake.deployIntake().until(intake.intakePivotAtPositionSetpoint()));
                NamedCommands.registerCommand("StopWithX", Commands.run(drive::stopWithX, drive));
                NamedCommands.registerCommand("IntakeRunRollers", intake.intakeRunRollers());
                NamedCommands.registerCommand("IntakeStopRollers", intake.intakeStopRollers());
                NamedCommands.registerCommand("ShooterSpinUp",
                                Commands.parallel(
                                                shooter.setShooterAutoVelocity(drive)
                                                                .until(shooter.shooterAtVelocitySetPoint()),
                                                hood.setHoodAutoAngle(drive).until(hood.hoodAtPositionSetpoint())));
                NamedCommands.registerCommand("AutoShooterSixSeconds", Commands.race(
                                Commands.parallel(
                                                shooter.setShooterAutoVelocity(drive),
                                                hood.setHoodAutoAngle(drive),
                                                Commands.repeatingSequence(Commands.race(
                                                                indexer.runIndexer(),
                                                                new WaitCommand(0.5)),
                                                                Commands.race(
                                                                                indexer.stopIndexer(),
                                                                                new WaitCommand(0.2))),
                                                kicker.runKicker()),
                                new WaitCommand(6.0)));
                NamedCommands.registerCommand("AutoShooterEndless", Commands.parallel(
                                shooter.setShooterAutoVelocity(drive),
                                hood.setHoodAutoAngle(drive),
                                Commands.repeatingSequence(Commands.race(
                                                indexer.runIndexer(),
                                                new WaitCommand(0.5)),
                                                Commands.race(
                                                                indexer.stopIndexer(),
                                                                new WaitCommand(0.2))),
                                kicker.runKicker()));
                NamedCommands.registerCommand("ShooterStop", Commands.parallel(
                                shooter.turnOffShooter(),
                                hood.retractHood(),
                                indexer.stopIndexer(),
                                kicker.stopKicker()));

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

                // Set up SysId routines
                //
                // Commented out to clean up autochooser
                //
                // autoChooser.addOption(
                // "Drive Wheel Radius Characterization",
                // DriveCommands.wheelRadiusCharacterization(drive));
                // autoChooser.addOption(
                // "Drive Simple FF Characterization",
                // DriveCommands.feedforwardCharacterization(drive));
                // autoChooser.addOption(
                // "Drive SysId (Quasistatic Forward)",
                // drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                // autoChooser.addOption(
                // "Drive SysId (Quasistatic Reverse)",
                // drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
                // autoChooser.addOption(
                // "Drive SysId (Dynamic Forward)",
                // drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
                // autoChooser.addOption(
                // "Drive SysId (Dynamic Reverse)",
                // drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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

                // Default commands
                //
                // The below subsystems default to "off"
                // shooter.setDefaultCommand(shooter.turnOffShooter());
                hood.setDefaultCommand(hood.retractHood());
                kicker.setDefaultCommand(kicker.stopKicker());
                indexer.setDefaultCommand(indexer.stopIndexer());
                //intake.setDefaultCommand(intake.intakeStopRollers());

        // Set the drivers movement for steering and driving on the driver joysticks
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX()));

        // Driver Controls

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
        driverController.leftTrigger().whileTrue(DriveCommands.joystickDriveAutoAim(drive,
                () -> -driverController.getLeftY(), () -> -driverController.getLeftX()));

        // Toggle intake when RB is pressed
        // driverController.rightBumper().debounce(0.1).onTrue(intake.toggleIntake());

        // Run intake rollers when LT is pressed
        driverController.rightTrigger().whileTrue(intake.intakeRunRollers());

        // Switch to X pattern when X button is pressed
        driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        driverController.rightBumper().onTrue(this.intake.toggleIntakePivot());

        // driverController.rightBumper().debounce(0.1).and(this.hood.hoodAtPositionSetpoint())

        // driverController.rightBumper().debounce(0.1).whileTrue(this.intake.toggleIntakePivot());

        // Shooting setpoints
        // Commented out while testing auto aim
        //
        // driverController.leftBumper().whileTrue(DriveCommands.driveToPose(drive, ()
        // -> Constants.PoseConstants.scorePosition3L));
        // driverController.x().whileTrue(DriveCommands.driveToPose(drive, () ->
        // Constants.PoseConstants.scorePosition2L));
        // driverController.a().whileTrue(DriveCommands.driveToPose(drive, () ->
        // Constants.PoseConstants.scorePosition1));
        // driverController.b().whileTrue(DriveCommands.driveToPose(drive, () ->
        // Constants.PoseConstants.scorePosition2R));
        // driverController.rightBumper().whileTrue(DriveCommands.driveToPose(drive, ()
        // -> Constants.PoseConstants.scorePosition3R));

        // Operator Controls

        // Run intake rollers when LT is pressed
        operatorController.leftTrigger().whileTrue(intake.intakeRunRollers());

        operatorController.rightBumper().whileTrue(Commands.parallel(
                shooter.setShooterVelocityPosition3(),
                hood.hoodToAngle(45),
                kicker.runKicker(),
                Commands.repeatingSequence(Commands.race(
                        indexer.runIndexer(),
                        new WaitCommand(0.5)),
                        Commands.race(
                                indexer.stopIndexer(),
                                new WaitCommand(0.2)))));

        // Auto speed and angle when RT is held
        operatorController.rightTrigger().whileTrue(
                Commands.parallel(
                        shooter.setShooterAutoVelocity(drive),
                        hood.setHoodAutoAngle(drive),
                        kicker.runKicker(),
                        Commands.repeatingSequence(Commands.race(
                                indexer.runIndexer(),
                                new WaitCommand(0.5)),
                                Commands.race(
                                        indexer.stopIndexer(),
                                        new WaitCommand(0.2)))));

        // Stop all subsystems (except drivetrain)
        operatorController.b().whileTrue(
                Commands.parallel(
                        shooter.turnOffShooter(),
                        kicker.stopKicker(),
                        indexer.stopIndexer(),
                        hood.retractHood(),
                        intake.intakeStopRollers()));

        // Resets cancoder to 0.12 rotations when start and back are pressed together,
        // must be used when intake is at upper hard limit
        operatorController.start().and(operatorController.back()).onTrue(intake.intakeResetCanCoder());

        // Shooting setpoints
        // Commented out while testing auto aim
        //
        operatorController.a().whileTrue(
                Commands.sequence(
                        Commands.parallel(
                                shooter.setShooterVelocityPosition1().until(
                                        shooter.shooterAtVelocitySetPoint()),
                                hood.hoodToAnglePosition1()
                                        .until(hood.hoodAtPositionSetpoint())),
                        Commands.parallel(
                                shooter.setShooterVelocityPosition1(),
                                hood.hoodToAnglePosition1(),
                                kicker.runKicker(),
                                Commands.repeatingSequence(Commands.race(
                                        indexer.runIndexer(),
                                        new WaitCommand(0.5)),
                                        Commands.race(
                                                indexer.stopIndexer(),
                                                new WaitCommand(0.2))))));

        operatorController.x().whileTrue(
                Commands.repeatingSequence(
                        Commands.race(
                                intake.intakePivotToAngle(Constants.IntakeConstants.intakePivotPulseUpAngleDegrees),
                                new WaitCommand(1)),
                        Commands.race(
                                intake.intakePivotToAngle(Constants.IntakeConstants.intakePivotDeployAngleDegrees),
                                new WaitCommand(1))));

        // operatorController.y().whileTrue(Commands.repeatingSequence(Commands.race(
        // indexer.runIndexer(),
        // new WaitCommand(0.5)),
        // Commands.race(
        // indexer.stopIndexer(),
        // new WaitCommand(0.2))));

        // operatorController.leftTrigger().whileTrue(
        // Commands.sequence(
        // Commands.parallel(
        // shooter.setShooterVelocityPosition2().until(shooter.shooterAtVelocitySetPoint()),
        // hood.hoodToAnglePosition2().until(hood.hoodAtPositionSetpoint())),
        // Commands.parallel(
        // kicker.turnOnKicker(),
        // indexer.turnOnIndexer())));

        // operatorController.rightTrigger().whileTrue(
        // Commands.sequence(
        // Commands.parallel(
        // shooter.setShooterVelocityPosition3().until(shooter.shooterAtVelocitySetPoint()),
        // hood.hoodToAnglePosition3().until(hood.hoodAtPositionSetpoint())),
        // Commands.parallel(
        // kicker.turnOnKicker(),
        // indexer.turnOnIndexer())));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        // return Autos.exampleAuto(m_exampleSubsystem);
        return Commands.sequence(
                Commands.print("Starting autonomous :: " + autoChooser.toString()),
                autoChooser.get(),
                Commands.print("Ending Autonomous"));
        // return autoChooser.get();
    }
}
