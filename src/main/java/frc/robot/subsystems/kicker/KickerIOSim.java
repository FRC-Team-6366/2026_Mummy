package frc.robot.subsystems.kicker;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class KickerIOSim implements KickerIO {
  FlywheelSim flywheelSim;
  DCMotor KICKER_GEARBOX = DCMotor.getKrakenX60(1);
  SimpleMotorFeedforward kickerFeedforward = new SimpleMotorFeedforward(
    0, 
    0.12, 
    0
  );
  private double volts = 0.0;

  public KickerIOSim() {
    this.flywheelSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            KICKER_GEARBOX,
            0.004,
            3),
        KICKER_GEARBOX, 0.004);
  }

  @Override
  public void setKickPower(double power) {
    double calcVolts = power * 12.0;
    calcVolts = MathUtil.clamp(calcVolts, -12.0, 12.0);
    this.volts = this.kickerFeedforward.calculate(calcVolts);
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    // Update the applied voltage to the motors and 
    this.flywheelSim.setInput(this.volts);
    this.flywheelSim.update(0.020);
    
    inputs.connected = true;
    inputs.kickCurrent = this.flywheelSim.getCurrentDrawAmps();
    inputs.kickRps = this.flywheelSim.getAngularVelocityRPM() / 60;
    inputs.kickSupplyCurrent = this.flywheelSim.getCurrentDrawAmps();
    inputs.kickVolts = this.flywheelSim.getInputVoltage();
  }

}
