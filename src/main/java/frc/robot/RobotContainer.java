// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.driveTrain.Drive;
import frc.robot.subsystems.driveTrain.DriveCommands;
import frc.robot.subsystems.driveTrain.GyroIO;
import frc.robot.subsystems.driveTrain.GyroIOPigeon2;
import frc.robot.subsystems.driveTrain.ModuleIO;
import frc.robot.subsystems.driveTrain.ModuleIOSim;
import frc.robot.subsystems.driveTrain.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    private ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

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

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption(
                "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption(
                "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        this.shooter = new Shooter(new ShooterIOTalonFX());// fixed an error when merging new shooter code
        this.indexer = new Indexer(new IndexerIOTalonFX());
        this.kicker = new Kicker(new KickerIOTalonFX());
        this.hood = new Hood(new HoodIOTalonFX());
        this.intake = new Intake(new IntakeIOTalonFX());
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

        // Default commands
        //
        // The below subsystems default to "off"
        shooter.setDefaultCommand(shooter.turnOffShooter());
        hood.setDefaultCommand(hood.retractHood());
        kicker.setDefaultCommand(kicker.stopKicker());
        indexer.setDefaultCommand(indexer.stopIndexer());
        intake.setDefaultCommand(intake.intakeStopRollers());

        // Set the drivers movement for steering and driving on the driver joysticks
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX()));
        
        // Driver Controls

        // Lock to Hub when RT is held
        driverController.rightTrigger().whileTrue(DriveCommands.joystickDriveAutoAim(drive, () -> -driverController.getLeftY(), () -> -driverController.getLeftX()));

        // Toggle intake when RB is pressed
        // driverController.rightBumper().debounce(0.1).onTrue(intake.toggleIntake());

        // Run intake rollers when LT is pressed
        driverController.leftTrigger().whileTrue(intake.intakeRunRollers());

        // Switch to X pattern when X button is pressed
        driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        driverController.rightBumper().onTrue(this.intake.toggleIntakePivot());

        // driverController.rightBumper().debounce(0.1).and(this.hood.hoodAtPositionSetpoint())

        // driverController.rightBumper().debounce(0.1).whileTrue(this.intake.toggleIntakePivot());
        
        
        // Shooting setpoints
        // Commented out while testing auto aim
        //
        // driverController.leftBumper().whileTrue(DriveCommands.driveToPose(drive, () -> Constants.PoseConstants.scorePosition3L));
        // driverController.x().whileTrue(DriveCommands.driveToPose(drive, () -> Constants.PoseConstants.scorePosition2L));
        // driverController.a().whileTrue(DriveCommands.driveToPose(drive, () -> Constants.PoseConstants.scorePosition1));
        // driverController.b().whileTrue(DriveCommands.driveToPose(drive, () -> Constants.PoseConstants.scorePosition2R)); 
        // driverController.rightBumper().whileTrue(DriveCommands.driveToPose(drive, () -> Constants.PoseConstants.scorePosition3R));
                

        // Operator Controls

        // Run intake rollers when LT is pressed
        operatorController.leftTrigger().whileTrue(intake.intakeRunRollers());

        // Auto speed and angle when RT is held
        operatorController.rightTrigger().whileTrue(Commands.sequence(
                Commands.parallel(
                        shooter.setShooterAutoVelocity(drive).until(shooter.shooterAtVelocitySetPoint()),
                        hood.setHoodAutoAngle(drive).until(hood.hoodAtPositionSetpoint())),
                Commands.parallel(
                        kicker.runKicker(),
                        indexer.runIndexer())));

        // Stop all subsystems (except drivetrain)
        operatorController.b().whileTrue(
                Commands.parallel(
                        shooter.turnOffShooter(),
                        kicker.stopKicker(),
                        indexer.stopIndexer(),
                        hood.retractHood(),
                        intake.intakeStopRollers()
                ));

        // Resets cancoder to 0.12 rotations when start and back are pressed together,
        // must be used when intake is at upper hard limit
        operatorController.start().and(operatorController.back()).onTrue(intake.intakeResetCanCoder());

        // Shooting setpoints
        // Commented out while testing auto aim
        //
        // operatorController.rightBumper().whileTrue(
        //         Commands.sequence(
        //                 Commands.parallel(
        //                         shooter.setShooterVelocityPosition1().until(shooter.shooterAtVelocitySetPoint()),
        //                         hood.hoodToAnglePosition1().until(hood.hoodAtPositionSetpoint())),
        //                 Commands.parallel(
        //                         kicker.turnOnKicker(),
        //                         indexer.turnOnIndexer())));

        // operatorController.leftTrigger().whileTrue(
        //         Commands.sequence(
        //                 Commands.parallel(
        //                         shooter.setShooterVelocityPosition2().until(shooter.shooterAtVelocitySetPoint()),
        //                         hood.hoodToAnglePosition2().until(hood.hoodAtPositionSetpoint())),
        //                 Commands.parallel(
        //                         kicker.turnOnKicker(),
        //                         indexer.turnOnIndexer())));

        // operatorController.rightTrigger().whileTrue(
        //         Commands.sequence(
        //                 Commands.parallel(
        //                         shooter.setShooterVelocityPosition3().until(shooter.shooterAtVelocitySetPoint()),
        //                         hood.hoodToAnglePosition3().until(hood.hoodAtPositionSetpoint())),
        //                 Commands.parallel(
        //                         kicker.turnOnKicker(),
        //                         indexer.turnOnIndexer())));

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
