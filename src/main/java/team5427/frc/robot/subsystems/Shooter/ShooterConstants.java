package team5427.frc.robot.subsystems.Shooter;

import team5427.lib.drivers.CANDeviceId;
import team5427.lib.drivers.ComplexGearRatio;
import team5427.lib.motors.MotorConfiguration;
import team5427.lib.motors.MotorUtil;
import team5427.lib.motors.MotorConfiguration.IdleState;
import team5427.lib.motors.MotorConfiguration.MotorMode;

public class ShooterConstants {
    public static CANDeviceId shooterMotorMasterCanDeviceId = new CANDeviceId(13);
    public static MotorConfiguration kShooterMotorMasterConfiguration = new MotorConfiguration();

    public static CANDeviceId shooterMotorSlaveCanDeviceId = new CANDeviceId(14);
    public static MotorConfiguration kShooterMotorSlaveConfiguration = new MotorConfiguration();

    static {
        kShooterMotorMasterConfiguration.maxVelocity = kShooterMotorMasterConfiguration.getStandardMaxVelocity(MotorUtil.kKrakenX60FOC_MaxRPM);
        kShooterMotorMasterConfiguration.maxAcceleration = kShooterMotorMasterConfiguration.maxVelocity * 50;
        kShooterMotorMasterConfiguration.gearRatio = new ComplexGearRatio(5, 20);
        kShooterMotorMasterConfiguration.isArm = false;
        kShooterMotorMasterConfiguration.idleState = IdleState.kBrake;
        kShooterMotorMasterConfiguration.mode = MotorMode.kFlywheel;
        kShooterMotorMasterConfiguration.withFOC = true;
        kShooterMotorMasterConfiguration.currentLimit = 46;
        kShooterMotorMasterConfiguration.kV = 0.7;
        kShooterMotorMasterConfiguration.kA = 0.1;
        kShooterMotorMasterConfiguration.kP = 0.5;
        kShooterMotorMasterConfiguration.kFF = 1;
    }

    static {
        kShooterMotorSlaveConfiguration.maxVelocity = kShooterMotorMasterConfiguration.maxVelocity;
        kShooterMotorSlaveConfiguration.maxAcceleration = kShooterMotorMasterConfiguration.maxAcceleration;
        kShooterMotorSlaveConfiguration.gearRatio = new ComplexGearRatio(5, 20);
        kShooterMotorSlaveConfiguration.isArm = false;
        kShooterMotorSlaveConfiguration.idleState = IdleState.kBrake;
        kShooterMotorSlaveConfiguration.mode = MotorMode.kFlywheel;
        kShooterMotorSlaveConfiguration.withFOC = true;
        kShooterMotorSlaveConfiguration.currentLimit = 46;
    }


}
