// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems.elevator;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import first.robot.hardware.Motors;
import org.wpilib.hardware.bus.CANPort;

/** Add your docs here. */
public class ElevatorConstants {
  public static final int MOTOR_ID = 14;
  public static final CANBus CAN_BUS = new CANBus(CANPort.CAN_S1);

  public static final int LIMIT_SWITCH_ID = 15;

  public static final double MAX_HEIGHT_ELEVATOR_M = 1;
  public static final double LOW_HEIGHT_ELEVATOR_M = 0;
  public static final double STAGE1_HEIGHT_M = 0.25;
  public static final double STAGE2_HEIGHT_M = 0.50;
  public static final double STAGE3_HEIGHT_M = 0.95;

  public static final double ELEVATOR_TOLERANCE = 0.01;
  public static final double MASS_KG = 5.0;

  public static final double ELEVATOR_LENGTH_METERS = 0.5;

  public static final double SWITCH_HEIGHT = 0;

  // 100 rotations = 1 m
  public static final double GEAR_RATIO = 100;
  public static final InvertedValue MOTOR_DIRECTION = InvertedValue.Clockwise_Positive;
  public static final Motors.Spec MOTOR = Motors.KRAKEN_X60;

  private static final double VOLTS_PER_NEWTON_METER =
      MOTOR.resistanceOhms() / MOTOR.torquePerAmp();

  private static final double GRAVITY = 9.81;

  public static final double ROT_TO_METERS = (2 * Math.PI * GEAR_RATIO);

  public static final double KG = MASS_KG * GRAVITY * VOLTS_PER_NEWTON_METER / ROT_TO_METERS;
  public static final double KV = GEAR_RATIO / MOTOR.rpsPerVolt();
  public static final double KA = MASS_KG * VOLTS_PER_NEWTON_METER / ROT_TO_METERS;
  public static final double KS = 0.0;
  public static final double KP = 50.0;
  public static final double KD = 0.0;

  public static final double CRUISE_VELOCITY_M = 0.5;
  public static final double ACCELERATION_M = 1.5;

  /** Starting values for the gains Arm allows tuning live. */
  public static final ElevatorIO.Gains GAINS =
      new ElevatorIO.Gains(
          KP, KD, KG, CRUISE_VELOCITY_M / ROT_TO_METERS, ACCELERATION_M / ROT_TO_METERS);

  public static final double STATOR_CURRENT_LIMIT_AMPS = 60.0;
}
