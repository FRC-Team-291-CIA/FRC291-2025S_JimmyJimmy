// Original Constants.java File From BroncBotz3481/YAGSL-Example Release 2025.7.2

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public final class Constants {

  public static class CodeMode {
    public enum CodeState {
      COMPETITION,
      PRACTICE;
    }

    public static final CodeState CODE_STATE = CodeState.PRACTICE;
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
    public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
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
    public static final double SPEED_FAST_MAX_TRANSITIONAL = 25.00;
    public static final double SPEED_FAST_MAX_ROTATIONAL = 8.00;

    public static final double SPEED_NORMAL_MAX_TRANSITIONAL = 20.00;
    public static final double SPEED_NORMAL_MAX_ROTATIONAL = 6.0;

    public static final double SPEED_SLOW_MAX_TRANSITIONAL = 4.00;
    public static final double SPEED_SLOW_MAX_ROTATIONAL = 3.00;

    public static final double SPEED_VERY_SLOW_MAX_TRANSITIONAL = 2.00;
    public static final double SPEED_VERY_SLOW_MAX_ROTATIONAL = 2.00;
  }

}
