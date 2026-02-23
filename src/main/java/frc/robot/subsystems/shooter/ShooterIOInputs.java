package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ShooterIOInputs {
    boolean connected = false;
    
    // For tracking Hardware statuses
    double leadShooterVolts = 0;
    double leadShooterPosition = 0;
    double leadShooterRps = 0;
    double leadShooterCurrent = 0;
    double leadShooterSupplyCurrent = 0;

    double followShooterVolts = 0;
    double followShooterPosition = 0;
    double followShooterRps = 0;
    double followShooterCurrent = 0;
    double followShooterSupplyCurrent = 0; 
    
    // For tracking Setpoint statuses
    double shooterVelocitySetpoint = 0;
    double shooterVelocityError = 0;
    boolean shooterAtVelocitySetpoint = false;
}
