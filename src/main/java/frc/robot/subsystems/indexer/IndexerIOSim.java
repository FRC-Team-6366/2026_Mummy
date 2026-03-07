package frc.robot.subsystems.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IndexerIOSim implements IndexerIO {
    FlywheelSim indexMotor;
    DCMotor INDEX_GEARBOX = DCMotor.getKrakenX60(1);
    private double volts = 0.0;

    public IndexerIOSim() {
        this.indexMotor = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                INDEX_GEARBOX, 
                1, 
                3
            ), INDEX_GEARBOX, 0.004);
    }

    @Override
    public void setIndexerPower(double power) {
        this.volts = power * 12;
        this.indexMotor.setInputVoltage(this.volts);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.connected = true;
        inputs.indexCurrent = this.indexMotor.getCurrentDrawAmps();
        inputs.indexRps = this.indexMotor.getAngularVelocityRPM()/60;
        inputs.indexSupplyCurrent = this.indexMotor.getCurrentDrawAmps();
        inputs.indexVolts = this.indexMotor.getInputVoltage();
    }
}
