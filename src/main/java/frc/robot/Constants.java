// Original Constants.java File From BroncBotz3481/YAGSL-Example Release 2025.7.2

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import swervelib.math.Matter;

public final class Constants {

  public static class CodeConstants {
    public enum CodeMode {
      COMPETITION,
      PRACTICE;
    }

    public static final CodeMode CODE_MODE = CodeMode.COMPETITION;

    public enum DevControllerMode {
      ON,
      OFF;
    }

    public static final DevControllerMode DEV_CONTROLLER_MODE = DevControllerMode.OFF;
  }

  public static class ControllerDriverConstants { // Logitech Gamepad F310 in D Mode
    // Controller USB port
    public static final int JOYSTICK_PORT = 0;

    // Joystick axis mappings
    public static final int AXIS_LEFT_Y = 1;
    public static final int AXIS_LEFT_X = 0;
    public static final int AXIS_RIGHT_Y = 3;
    public static final int AXIS_RIGHT_X = 2;

    // Button mappings
    public static final int BUTTON_A = 2;
    public static final int BUTTON_B = 3;
    public static final int BUTTON_Y = 4;
    public static final int BUTTON_X = 1;

    public static final int BUTTON_BACK = 9;
    public static final int BUTTON_START = 10;

    public static final int BUTTON_JOYSTICK_LEFT = 11;
    public static final int BUTTON_JOYSTICK_RIGHT = 12;

    public static final int BUTTON_BUMPER_TOP_LEFT = 5;
    public static final int BUTTON_BUMPER_TOP_RIGHT = 6;

    public static final int BUTTON_BUMPER_BOTTOM_LEFT = 7;
    public static final int BUTTON_BUMPER_BOTTOM_RIGHT = 8;

    // Deadband settings for joystick input
    public static final double DEADBAND = 0.05;
  }

  public static class ControllerOperatorConstants { // Logitech Gamepad F310 in D Mode
    // Controller USB port
    public static final int JOYSTICK_PORT = 1;

    // Joystick axis mappings
    public static final int AXIS_LEFT_Y = 1;
    public static final int AXIS_LEFT_X = 0;
    public static final int AXIS_RIGHT_Y = 3;
    public static final int AXIS_RIGHT_X = 2;

    // Button mappings
    public static final int BUTTON_A = 2;
    public static final int BUTTON_B = 3;
    public static final int BUTTON_Y = 4;
    public static final int BUTTON_X = 1;

    public static final int BUTTON_BACK = 9;
    public static final int BUTTON_START = 10;

    public static final int BUTTON_JOYSTICK_LEFT = 11;
    public static final int BUTTON_JOYSTICK_RIGHT = 12;

    public static final int BUTTON_BUMPER_TOP_LEFT = 5;
    public static final int BUTTON_BUMPER_TOP_RIGHT = 6;

    public static final int BUTTON_BUMPER_BOTTOM_LEFT = 7;
    public static final int BUTTON_BUMPER_BOTTOM_RIGHT = 8;

    // Deadband settings for joystick input
    public static final double DEADBAND = 0.05;
  }

  public static final class YAGSLConstants {
    // FRC 291: From Based Constants.java
    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms spark max velocity lag
    public static final double MAX_SPEED = Units.feetToMeters(14.5);

    // FRC 291: From Based Constants.java in DrivebaseConstants class
    public static final double WHEEL_LOCK_TIME = 10; // seconds

    // FRC 291: From Based Constants.java in OperatorConstants class
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static final class SwerveConstants {
    public static final double SPEED_FAST_MAX_TRANSITIONAL = 30.00;
    public static final double SPEED_FAST_MAX_ROTATIONAL = 10.00;

    public static final double SPEED_NORMAL_MAX_TRANSITIONAL = 25.00;
    public static final double SPEED_NORMAL_MAX_ROTATIONAL = 8.0;

    public static final double SPEED_SLOW_MAX_TRANSITIONAL = 4.00;
    public static final double SPEED_SLOW_MAX_ROTATIONAL = 3.00;

    public static final double SPEED_VERY_SLOW_MAX_TRANSITIONAL = 2.00;
    public static final double SPEED_VERY_SLOW_MAX_ROTATIONAL = 2.00;
  }

  public static final class ElevatorConstants {
    // Motor configuration for elevator
    public static final int MOTOR_LEFT_CANID = 20;
    public static final MotorType MOTOR_LEFT_TYPE = MotorType.kBrushless;
    public static final IdleMode MOTOR_LEFT_MODE = IdleMode.kBrake;
    public static final boolean MOTOR_LEFT_IS_INVERTED = true;

    public static final int MOTOR_RIGHT_CANID = 21;
    public static final MotorType MOTOR_RIGHT_TYPE = MotorType.kBrushless;
    public static final IdleMode MOTOR_RIGHT_MODE = IdleMode.kBrake;
    public static final boolean MOTOR_RIGHT_IS_INVERTED = true;

    // Conversion factor for motor revolutions per inch of elevator travel
    public static final double MOTOR_REVOLUTION_PER_INCH = (4335.00 / 4698.00);

    // Current limit for smart motor control
    public static final int MOTOR_SMART_CURRENT_LIMIT = 40;

    // PID slot 0 constants for motor control up
    public static final double SLOT_ZERO_P = 0.10;
    public static final double SLOT_ZERO_I = 0.00;
    public static final double SLOT_ZERO_D = 0.00;
    public static final double SLOT_ZERO_FF = 0.00;
    public static final ArbFFUnits SLOT_ZERO_FF_UNITS = ArbFFUnits.kVoltage;

    // PID slot 1 constants for motor control down
    public static final double SLOT_ONE_P = 0.025;
    public static final double SLOT_ONE_I = 0.00;
    public static final double SLOT_ONE_D = 0.00;
    public static final double SLOT_ONE_FF = 0.00;
    public static final ArbFFUnits SLOT_ONE_FF_UNITS = ArbFFUnits.kVoltage;

    // Predefined elevator height positions for different levels
    public static final double HEIGHT_CORAL_LEVEL_FOUR = -53.00; // Estimated based on manual lift.
    public static final double HEIGHT_CORAL_LEVEL_THREE = -25.00; // Slightly lower than 25 for best fit.
    public static final double HEIGHT_CORAL_LEVEL_TWO = -10.00; // Slightly lower than 10 for best fit.
    public static final double HEIGHT_CORAL_LEVEL_ONE = -5.00; // Unknown value.
    public static final double HEIGHT_CORAL_INTAKE = -0.25; // Unknown value.
    public static final double HEIGHT_PARK = 0.00;
    public static final double HEIGHT_KILL_POWER = Double.NaN;
    public static final double HEIGHT_DISABLED = Double.NaN;
    public static final double HEIGHT_TEST = Double.NaN;
  }

  public static final class CoralConstants {
    // Motor configuration for coral mechanism
    public static final int MOTOR_LEFT_CANID = 22;
    public static final MotorType MOTOR_LEFT_TYPE = MotorType.kBrushed;
    public static final IdleMode MOTOR_LEFT_MODE = IdleMode.kBrake;
    public static final boolean MOTOR_LEFT_IS_INVERTED = false;

    public static final int MOTOR_RIGHT_CANID = 23;
    public static final MotorType MOTOR_RIGHT_TYPE = MotorType.kBrushed;
    public static final IdleMode MOTOR_RIGHT_MODE = IdleMode.kBrake;
    public static final boolean MOTOR_RIGHT_IS_INVERTED = true;

    // Sensor configuration for coral intake detection
    public static final int INTAKE_SENSOR_DIO_PORT = 0;
    public static final boolean INTAKE_SENSOR_IS_INVERTED = true;

    // Manual control speeds for intake system
    public static final double SPEED_MANUAL_FORWARD_SLOW = 0.3;
    public static final double SPEED_MANUAL_FORWARD_FAST = 0.4;
    public static final double SPEED_MANUAL_REVERSE_SLOW = -0.3;
    public static final double SPEED_MANUAL_REVERSE_FAST = -0.4;
  }

  public static final class FlapConstants {

    public enum FlapControlMode {
      ANGLE,
      MANUAL;
    }

    public static final FlapControlMode FLAP_CONTROL_MODE = FlapControlMode.MANUAL;

    public static final int MOTOR_CANID = 24;
    public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
    public static final IdleMode MOTOR_MODE = IdleMode.kBrake;
    public static final boolean MOTOR_IS_INVERTED = false;
    public static final int MOTOR_SMART_CURRENT_LIMIT = 40;
    public static final double MOTOR_REVOLUTION_PER_INCH = 360;

    public static final double SLOT_ZERO_P = 0.50;
    public static final double SLOT_ZERO_I = 0.00;
    public static final double SLOT_ZERO_D = 0.00;
    public static final double SLOT_ZERO_FF = 0.00;
    public static final ArbFFUnits SLOT_ZERO_FF_UNITS = ArbFFUnits.kPercentOut;

    public static final double ANGLE_UP = 135;
    public static final double ANGLE_LEVEL = 90;
    public static final double ANGLE_DOWN = 0;
    public static final double ANGLE_DISABLED = Double.NaN;

    public static final double UP_SPEED = 0.5;
    public static final double DOWN_SPEED = -0.5;
  }

  public static final class ClimberConstants {
    public static final int MOTOR_CANID = 25;
    public static final MotorType MOTOR_TYPE = MotorType.kBrushless;
    public static final IdleMode MOTOR_MODE = IdleMode.kBrake;
    public static final boolean MOTOR_IS_INVERTED = false;
    public static final int MOTOR_SMART_CURRENT_LIMIT = 40;

    public static final double SPEED_IN = 1;
    public static final double SPEED_OUT = -1;
  }

  public static final class LedConstants {
    public static final int CANID = 19;
  }

  public static class CIAAutoConstants {
    // Used in IntakeCoralCommand.java
    public static final double AUTO_SPEED_CORAL_BEFORE_ENTER = 0.5;
    public static final double AUTO_SPEED_CORAL_AFTER_ENTER = 0.25;
  }

}
