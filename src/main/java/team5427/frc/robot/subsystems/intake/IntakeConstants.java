package team5427.frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import team5427.lib.drivers.CANDeviceId;
import team5427.lib.drivers.ComplexGearRatio;
import team5427.lib.motors.MotorConfiguration;
import team5427.lib.motors.MotorConfiguration.IdleState;
import team5427.lib.motors.MotorConfiguration.MotorMode;
import team5427.lib.motors.MotorUtil;

public final class IntakeConstants {
  public static MotorConfiguration kRollerMotorConfiguration = new MotorConfiguration();

  public static final ComplexGearRatio kRollerMotorGearRatio = new ComplexGearRatio(1.0);

  public static final CANDeviceId kRollerMotorCanId = new CANDeviceId(13);

  static {
    kRollerMotorConfiguration.gearRatio = kRollerMotorGearRatio;
    kRollerMotorConfiguration.finalDiameterMeters = Units.inchesToMeters(2.0);
    kRollerMotorConfiguration.isArm = false;
    kRollerMotorConfiguration.idleState = IdleState.kBrake;
    kRollerMotorConfiguration.isInverted = false; // CCW is +, CW is -
    kRollerMotorConfiguration.mode = MotorMode.kFlywheel;
    kRollerMotorConfiguration.withFOC = true;

    kRollerMotorConfiguration.maxVelocity =
        kRollerMotorConfiguration.getStandardMaxVelocity(MotorUtil.kNEO_MaxRPM);
    kRollerMotorConfiguration.maxAcceleration = kRollerMotorConfiguration.maxVelocity * 2.0;

    kRollerMotorConfiguration.kP = 1.0;
    kRollerMotorConfiguration.kI = 0.0;
    kRollerMotorConfiguration.kD = 0.0;

    kRollerMotorConfiguration.kV = 0.0;
    kRollerMotorConfiguration.kA = 0.0;
    kRollerMotorConfiguration.kS = 0.0;
    kRollerMotorConfiguration.kG = 1.0;
    kRollerMotorConfiguration.kFF = 0.0;

    kRollerMotorConfiguration.currentLimit = 60;
  }

  public static final double kRollerMotorSimulatedkP = 0.2;

  public static final LinearVelocity kRollerStowedVelocity = MetersPerSecond.of(0.1);
  public static final LinearVelocity kRollerIntakeVelocity = MetersPerSecond.of(5.0);
}
