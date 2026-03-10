package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
    FlywheelSim leadShooterMotor;
    DCMotor LEAD_SHOOTER_GEARBOX = DCMotor.getKrakenX60(1);
    public static final double shooterMinVelocityRPS = 0;
    public static final double shooterMaxVelocityRPS = 100;
    private double rps = 0.0;
    private double radPerSeconds = 0.0;
    private double setpointThreshold = 1;


    public ShooterIOSim() {
        this.leadShooterMotor = new FlywheelSim(LinearSystemId.createFlywheelSystem(LEAD_SHOOTER_GEARBOX, 12,1), LEAD_SHOOTER_GEARBOX, 0.004);
    }

    @Override
    public void setShooterVelocityFeetPerSecond(double feetPerSecond) {
        double rotationsPerSecond = feetPerSecond / ((4.0 / 12.0) * Math.PI);
        this.rps = MathUtil.clamp(rotationsPerSecond, shooterMinVelocityRPS, shooterMaxVelocityRPS);
        this.radPerSeconds = this.rps*2*Math.PI;
        this.leadShooterMotor.setAngularVelocity(radPerSeconds);
    }

    @Override
    public double getShooterVelocityError() {
        return this.rps - this.leadShooterMotor.getAngularVelocityRPM()/60;
    }

    @Override
    public boolean shooterAtVelocitySetPoint() {
        return Math.abs(this.getShooterVelocityError()) < this.setpointThreshold;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.connected = true;
        inputs.leadShooterCurrent = this.leadShooterMotor.getCurrentDrawAmps();
        inputs.leadShooterPosition = Double.POSITIVE_INFINITY;
        inputs.leadShooterRps = this.leadShooterMotor.getAngularVelocityRPM()/60;
        inputs.leadShooterSupplyCurrent = this.leadShooterMotor.getCurrentDrawAmps();
        inputs.leadShooterVolts = this.leadShooterMotor.getInputVoltage();
        inputs.followShooterCurrent = this.leadShooterMotor.getCurrentDrawAmps();
        inputs.followShooterPosition = Double.POSITIVE_INFINITY;
        inputs.followShooterRps = this.leadShooterMotor.getAngularVelocityRPM()/60;
        inputs.followShooterSupplyCurrent = this.leadShooterMotor.getCurrentDrawAmps();
        inputs.followShooterVolts = this.leadShooterMotor.getInputVoltage();
        inputs.shooterVelocitySetpoint = this.leadShooterMotor.getAngularVelocityRPM();
        inputs.shooterVelocityError = this.getShooterVelocityError();
        inputs.shooterAtVelocitySetpoint = this.shooterAtVelocitySetPoint();
    }

}
