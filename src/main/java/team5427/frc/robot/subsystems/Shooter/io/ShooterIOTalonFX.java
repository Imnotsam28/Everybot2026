package team5427.frc.robot.subsystems.Shooter.io;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import team5427.frc.robot.Constants;
import team5427.frc.robot.subsystems.Shooter.ShooterConstants;
import team5427.frc.robot.subsystems.Shooter.io.ShooterIO.ShooterIOInputs;
import team5427.lib.motors.SteelTalonFX;

public class ShooterIOTalonFX implements ShooterIO {
    private SteelTalonFX shooterMotorMaster;
    private SteelTalonFX shooterMotorSlave;

    private LinearVelocity ShooterMotorMasterVelocity;
    private StatusSignal<AngularVelocity> ShooterMotorMasterAngularVelocity;
    private StatusSignal<Current> ShooterMotorMasterCurrent;
    private StatusSignal<Temperature> ShooterMotorMasterTemperature;
    private StatusSignal<Voltage> ShooterMotorMasterVoltage;
    private StatusSignal<AngularAcceleration> ShooterMotorMasterAngularAcceleration;
    private boolean isShooterMotorSlaveDisabled = false;

    private LinearVelocity ShooterMotorSlaveVelocity;
    private StatusSignal<AngularVelocity> ShooterMotorSlaveAngularVelocity;
    private StatusSignal<Current> ShooterMotorSlaveCurrent;
    private StatusSignal<Temperature> ShooterMotorSlaveTemperature;
    private StatusSignal<Voltage> ShooterMotorSlaveVoltage;
    private StatusSignal<AngularAcceleration> ShooterMotorSlaveAngularAcceleration;
    private boolean isShooterMotorMasterDisabled = false;

    public ShooterIOTalonFX() {
        shooterMotorMaster = new SteelTalonFX(ShooterConstants.shooterMotorMasterCanDeviceId);
        shooterMotorSlave = new SteelTalonFX(ShooterConstants.shooterMotorSlaveCanDeviceId);

        shooterMotorMaster.apply(ShooterConstants.kShooterMotorMasterConfiguration);
        shooterMotorSlave.apply(ShooterConstants.kShooterMotorSlaveConfiguration);

        shooterMotorMaster.setEncoderPosition(0.0);
        shooterMotorSlave.setEncoderPosition(0.0);

        ShooterMotorMasterAngularVelocity = shooterMotorMaster.getTalonFX().getVelocity();
        ShooterMotorSlaveAngularVelocity = shooterMotorSlave.getTalonFX().getVelocity();

        ShooterMotorMasterAngularAcceleration = shooterMotorMaster.getTalonFX().getAcceleration();
        ShooterMotorSlaveAngularAcceleration = shooterMotorSlave.getTalonFX().getAcceleration();

        ShooterMotorMasterCurrent = shooterMotorMaster.getTalonFX().getStatorCurrent();
        ShooterMotorSlaveCurrent = shooterMotorSlave.getTalonFX().getStatorCurrent();

        ShooterMotorMasterTemperature = shooterMotorMaster.getTalonFX().getDeviceTemp();
        ShooterMotorSlaveTemperature = shooterMotorSlave.getTalonFX().getDeviceTemp();

        ShooterMotorMasterVoltage = shooterMotorMaster.getTalonFX().getMotorVoltage();
        ShooterMotorSlaveVoltage = shooterMotorSlave.getTalonFX().getMotorVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(
            Constants.kHighPriorityUpdateFrequency, ShooterMotorMasterAngularVelocity);

        BaseStatusSignal.setUpdateFrequencyForAll(
            Constants.kMediumPriorityUpdateFrequency,
            ShooterMotorMasterAngularAcceleration,
            ShooterMotorSlaveAngularAcceleration,
            ShooterMotorMasterCurrent,
            ShooterMotorSlaveCurrent);

        BaseStatusSignal.setUpdateFrequencyForAll(
            Constants.kLowPriorityUpdateFrequency,
            ShooterMotorMasterTemperature,
            ShooterMotorSlaveTemperature,
            ShooterMotorMasterVoltage,
            ShooterMotorSlaveVoltage);
    }

    public void updateInputs(ShooterIOInputs inputs) {
        inputs.ShooterMotorMasterConnected = shooterMotorMaster.getTalonFX().isConnected();
        inputs.ShooterMotorSlaveConnected = shooterMotorSlave.getTalonFX().isConnected();

        inputs.ShooterMotorMasterDisabled = isShooterMotorMasterDisabled;
        inputs.ShooterMotorSlaveDisabled = isShooterMotorSlaveDisabled;

        BaseStatusSignal.refreshAll(ShooterMotorMasterAngularVelocity, ShooterMotorSlaveAngularVelocity);

        BaseStatusSignal.refreshAll(
            ShooterMotorMasterAngularAcceleration,
            ShooterMotorMasterCurrent,
            ShooterMotorSlaveAngularAcceleration,
            ShooterMotorSlaveCurrent
        );

        BaseStatusSignal.refreshAll(
            ShooterMotorMasterTemperature, ShooterMotorMasterVoltage, ShooterMotorSlaveTemperature, ShooterMotorMasterTemperature
        );

        inputs.ShooterMotorMasterAngularVelocity = ShooterMotorMasterAngularVelocity.getValue();
        inputs.ShooterMotorMasterAngularAcceleration = ShooterMotorMasterAngularAcceleration.getValue();

    }
}
