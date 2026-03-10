package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Indexer extends SubsystemBase {
  /**
   * Holds the current value of power that is mainly used by increment/decrement
   * methods. Its range is from 0.0 to 1.0
   */
  private double power = 0;
  IndexerIO indexerIO;

  /**
   * IOInputs object for holding current values for the devices of the
   * Indexer Subsystem. These values are updated every loop and written
   * to the Log for reviewing or replaying
   */
  IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  /**
   * Creates Indexer subsystem by using the supplied IndexerIO object that
   * represents
   * the underlying hardware for the Indexer subsystem
   * <p>
   * Example use:
   * 
   * <pre>{@code
   * Indexer indexer = new Indexer(new IndexerIOTalonFX());
   * }</pre>
   * 
   * @param io Hardware object that implements the IndexerIO interface class
   * 
   */
  public Indexer(IndexerIO io) {
    this.indexerIO = io;
  }

  /**
   * Turns off the Indexer subsystem's motors
   * <p>
   * Example use:
   * 
   * <pre>{@code
   * controller.b().onTrue(indexer.stopIndexer());
   * }</pre>
   * 
   * @return Command to turn off indexer motors
   */
  public Command stopIndexer() {
    return this.run(
        () -> {
          this.power = 0;
          this.indexerIO.setIndexerPower(this.power);
        }).withName("stopIndexer()");
  }

  /**
   * Turns on the Indexer subsystem's motors
   * <p>
   * Example use:
   * 
   * <pre>{@code
   * controller.a().whileTrue(indexer.runIndexer());
   * }</pre>
   * 
   * @return Command to turn on indexer motors
   */
  public Command runIndexer() {
    return this.run(
        () -> {
          this.power = 1;
          this.indexerIO.setIndexerPower(this.power);
        }).withName("runIndexer()");
  }

  /**
   * Increases the Indexer's subsystem's output by 0.2 to a maximum power
   * of 1.
   * <p>
   * Example use:
   * 
   * <pre>{@code
   * controller.leftTrigger().whileTrue(indexer.incrementIndexer());
   * }</pre>
   * 
   * @return Command to increase the indexer subsystem output by 0.2
   */
  public Command incrementIndexer() {
    return this.runOnce(
        () -> {
          this.power = MathUtil.clamp(this.power += 0.2, 0, 1);
          this.indexerIO.setIndexerPower(this.power);
          // this.indexerIO.setIndexerWallPower(this.power);
        }).withName("incrementIndexer()");
  }

  /**
   * Decreases the Indexer's subsystem's output by 0.2 to a minimum power
   * of 1.
   * <p>
   * Example use:
   * 
   * <pre>{@code
   * controller.leftTrigger().whileTrue(indexer.decrementIndexer());
   * }</pre>
   * 
   * @return Command to decrease the indexer subsystem output by 0.2
   */
  public Command decrementIndexer() {
    return this.runOnce(
        () -> {
          this.power = MathUtil.clamp(this.power -= 0.2, 0, 1);
          this.indexerIO.setIndexerPower(this.power);
          // this.indexerIO.setIndexerWallPower(this.power);
        }).withName("decrementIndexer()");
  }

  public Command pulseIndexer() {
    return Commands.repeatingSequence(
        Commands.race(
            this.runIndexer(),
            new WaitCommand(1)),
        Commands.race(
            this.stopIndexer(),
            new WaitCommand(0.5)))
        .withName("pulseIndexer()");
  }

  @Override
  public void periodic() {
    this.indexerIO.updateInputs(inputs);
    Logger.processInputs("Indexer Subsystem", inputs);
    Logger.recordOutput("IndexerSubsystem/DefaultCommand",
        this.getDefaultCommand() != null ? this.getDefaultCommand().getName() : "N/A");
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
