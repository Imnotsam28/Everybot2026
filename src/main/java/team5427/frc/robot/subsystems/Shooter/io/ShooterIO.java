package team5427.frc.robot.subsystems.Shooter.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.checkerframework.checker.units.qual.degrees;
import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public LinearVelocity ShooterMotorMasterLinearVelocity = MetersPerSecond.of(0.0);
        public AngularVelocity ShooterMotorMasterAngularVelocity = RotationsPerSecond.of(0.0);
        public AngularAcceleration shooterMotorMasterAngularAcceleration = RotationsPerSecondPerSecond.of(0.0);
        public LinearAcceleration ShooterMotorMasterAcceleration = MetersPerSecondPerSecond.of(0.0);
        public Voltage ShooterMotorMasterVoltage = Volts.of(0.0);
        public Current ShooterMotorMasterCurrent = Amps.of(0.0);
        public Temperature ShooterMotorMasterTemperature = Celsius.of(0.0);
        public boolean ShooterMotorMasterConnected = false;
        public boolean ShooterMotorMasterDisabled = false;

        public LinearVelocity ShooterMotorSlaveLinearVelocity = MetersPerSecond.of(0.0);
        public AngularVelocity ShooterMotorSlaveAngularVelocity = RotationsPerSecond.of(0.0);
        public AngularAcceleration shooterMotorSlaveAngularAcceleration = RotationsPerSecondPerSecond.of(0.0);
        public LinearAcceleration ShooterMotorSlaveAcceleration = MetersPerSecondPerSecond.of(0.0);
        public Voltage ShooterMotorSlaveVoltage = Volts.of(0.0);
        public Current ShooterMotorSlaveCurrent = Amps.of(0.0);
        public Temperature ShooterMotorSlaveTemperature = Celsius.of(0.0);
        public boolean ShooterMotorSlaveConnected = false;
        public boolean ShooterMotorSlaveDisabled = false;
    }
    
    public default void updateInputs(ShooterIOInputs inputs){
        
    }

    public default void setShooterSpeed(LinearVelocity velocity){

    }

    public default void setShooterSpeed(AngularVelocity velocity) {

    }

    public default void setShooterAcceleration(LinearAcceleration acceleration) {

    }

    public default void setShooterVoltage(Voltage voltage) {

    }

    public default void setShooterCurrent(Current current) {

    }

    public default void disableShooterMotor(boolean shouldDisable) {
        
    }
}

