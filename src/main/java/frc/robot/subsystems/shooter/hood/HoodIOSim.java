package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HoodIOSim implements HoodIO {
    private DCMotorSim hoodMotor;
    private DCMotor HOOD_GEARBOX = DCMotor.getKrakenX60(1);
    static final double hoodMinPosition = 0;
    static final double hoodMaxPosition = 5.6;
    private double angleRad = 0.0;
    private double setpointThreshold = (5.6 / 100) * 2;

    public HoodIOSim() {
        this.hoodMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(HOOD_GEARBOX, 1, 1), HOOD_GEARBOX, 0.004, 0.04);
        this.hoodMotor.setAngle(Units.degreesToRadians(15));
    }

    @Override
    public Rotation2d getRotations() {
        return Rotation2d.fromRadians(this.hoodMotor.getAngularPositionRad());
    }

    @Override
    public void hoodToAngle(double angle) {
        this.angleRad = angle * 2 * Math.PI;
        this.hoodMotor.setAngle(this.angleRad);
    }

    @Override
    public void hoodToPosition(double position) {
        this.angleRad = (MathUtil.clamp(position, 0, 5.6) * (30 / hoodMaxPosition) + 15) * 2 * Math.PI;
        this.hoodMotor.setAngle(this.angleRad );
    }

    @Override
    public double getHoodPositionError() {
        return this.angleRad - this.hoodMotor.getAngularPositionRad();
    }

    @Override
    public boolean hoodAtPositionSetpoint() {
        return Math.abs(this.getHoodPositionError()) < this.setpointThreshold;
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.connected = true;
        inputs.hoodCurrent = this.hoodMotor.getCurrentDrawAmps();
        inputs.hoodPosition = this.hoodMotor.getAngularPositionRotations();
        inputs.hoodPositionError = this.getHoodPositionError();
        inputs.hoodPositionSetpoint = Units.radiansToRotations(angleRad);
        inputs.hoodRps = this.hoodMotor.getAngularVelocityRPM()/60;
        inputs.hoodSupplyCurrent = this.hoodMotor.getCurrentDrawAmps();
        inputs.hoodVolts = this.hoodMotor.getInputVoltage();
        inputs.hoodAtSetpoint = this.hoodAtPositionSetpoint();
    }
}
