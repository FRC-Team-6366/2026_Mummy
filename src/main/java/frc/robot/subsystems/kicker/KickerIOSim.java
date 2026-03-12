package frc.robot.subsystems.kicker;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class KickerIOSim implements KickerIO {
  FlywheelSim kickerMotor;
  DCMotor KICKER_GEARBOX = DCMotor.getKrakenX60(1);
  private double volts = 0.0;

  public KickerIOSim() {
    this.kickerMotor = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            KICKER_GEARBOX,
            1,
            3),
        KICKER_GEARBOX, 0.004);
  }

  @Override
  public void setKickPower(double power) {
    this.volts = power * 12;
    this.kickerMotor.setInputVoltage(this.volts);
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.connected = true;
    inputs.kickCurrent = this.kickerMotor.getCurrentDrawAmps();
    inputs.kickPosition = Double.POSITIVE_INFINITY;
    inputs.kickRps = this.kickerMotor.getAngularVelocityRPM() / 60;
    inputs.kickSupplyCurrent = this.kickerMotor.getCurrentDrawAmps();
    inputs.kickVolts = this.kickerMotor.getInputVoltage();
  }

}
