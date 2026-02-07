package team5427.frc.robot.subsystems.intake.io;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import team5427.frc.robot.Constants;
import team5427.frc.robot.subsystems.intake.IntakeConstants;
import team5427.lib.motors.SteelTalonFX;

public class IntakeIOTalonFX implements IntakeIO {
  private SteelTalonFX rollerMotor;

  private StatusSignal<AngularVelocity> rollerMotorAngularVelocity;
  private StatusSignal<AngularAcceleration> rollerMotorAngularAcceleration;

  private StatusSignal<Current> rollerMotorCurrent;
  private StatusSignal<Voltage> rollerMotorVoltage;
  private StatusSignal<Temperature> rollerMotorTemperature;

  private boolean isRollerMotorDisabled = false;

  public IntakeIOTalonFX() {
    rollerMotor = new SteelTalonFX(IntakeConstants.kRollerMotorCanId);

    rollerMotor.apply(IntakeConstants.kRollerMotorConfiguration);

    rollerMotor.setEncoderPosition(0.0);

    rollerMotorAngularVelocity = rollerMotor.getTalonFX().getVelocity();
    rollerMotorAngularAcceleration = rollerMotor.getTalonFX().getAcceleration();

    rollerMotorCurrent = rollerMotor.getTalonFX().getStatorCurrent();
    rollerMotorVoltage = rollerMotor.getTalonFX().getMotorVoltage();
    rollerMotorTemperature = rollerMotor.getTalonFX().getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.kHighPriorityUpdateFrequency, rollerMotorAngularVelocity);

    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.kMediumPriorityUpdateFrequency,
        rollerMotorAngularAcceleration,
        rollerMotorCurrent);

    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.kLowPriorityUpdateFrequency,
        rollerMotorTemperature,
        rollerMotorVoltage);
  }

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollerMotorConnected = rollerMotor.getTalonFX().isConnected();

    inputs.rollerMotorDisabled = isRollerMotorDisabled;

    BaseStatusSignal.refreshAll(rollerMotorAngularVelocity);

    BaseStatusSignal.refreshAll(
        rollerMotorAngularAcceleration,
        rollerMotorCurrent);

    BaseStatusSignal.refreshAll(
        rollerMotorTemperature, 
        rollerMotorVoltage);

    inputs.rollerMotorAngularVelocity = rollerMotorAngularVelocity.getValue();
    inputs.rollerMotorAngularAcceleration = rollerMotorAngularAcceleration.getValue();
    inputs.rollerMotorCurrent = rollerMotorCurrent.getValue();
    inputs.rollerMotorLinearVelocity =
        MetersPerSecond.of(rollerMotor.getEncoderVelocity(rollerMotorAngularVelocity));
    inputs.rollerMotorLinearAcceleration =
        MetersPerSecondPerSecond.of(
            rollerMotor.getEncoderAcceleration(rollerMotorAngularAcceleration));
    inputs.rollerMotorTemperature = rollerMotorTemperature.getValue();
    inputs.rollerMotorVoltage = rollerMotorVoltage.getValue();
  }

  public void setRollerSpeed(LinearVelocity velocity) {
    if (!isRollerMotorDisabled) {
      rollerMotor.setSetpoint(velocity);
    }
  }

  public void setRollerSpeed(AngularVelocity velocity) {
    if (!isRollerMotorDisabled) {
      rollerMotor.setSetpoint(velocity);
    }
  }

  public void disableRollerMotor(boolean shouldDisable) {
    isRollerMotorDisabled = shouldDisable;
    if (shouldDisable) rollerMotor.setRawPercentage(0);
  }
}
