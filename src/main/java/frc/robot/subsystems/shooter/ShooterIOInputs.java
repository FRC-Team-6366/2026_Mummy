package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ShooterIOInputs {
    boolean connected = false;
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

    double hoodVolts = 0;
    double hoodPosition = 0;
    double hoodRps = 0;
    double hoodCurrent = 0;
    double hoodSupplyCurrent = 0;
    // double hoodCANPositionRotations = 0;
    // double hoodCANVelocityRps = 0;
}
