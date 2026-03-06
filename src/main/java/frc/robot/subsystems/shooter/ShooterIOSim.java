package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
    FlywheelSim leadShooterMotor;
    DCMotor LEAD_SHOOTER_GEARBOX = DCMotor.getKrakenX60(1);
    FlywheelSim followerShooterMotor;
    DCMotor FOLLOWER_SHOOTER_GEARBOX = DCMotor.getKrakenX60(1);


    public ShooterIOSim() {
        this.leadShooterMotor = new FlywheelSim(LinearSystemId.createFlywheelSystem(LEAD_SHOOTER_GEARBOX, 12,1), LEAD_SHOOTER_GEARBOX, 0.004);
        // (DCMotor.getKrakenX60(1), 1, 0.004);
    }

    @Override
    public void setShooterVelocityRPS(double rps) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setShooterVelocityRPS'");
    }

    @Override
    public void setShooterVelocityFeetPerSecond(double feetPerSecond) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setShooterVelocityFeetPerSecond'");
    }

    @Override
    public double getShooterVelocityError() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getShooterVelocityError'");
    }

    @Override
    public boolean shooterAtVelocitySetPoint() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'shooterAtVelocitySetPoint'");
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
    }

}
