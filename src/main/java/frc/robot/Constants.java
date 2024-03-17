// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }


  /**
   * If a joystick is plugged in, we assume it is in this port
   * 
   * <p>
   * WARNING: If using an Xbox controller and a joystick at the same time,
   * plug the joystick in second
   * </p>
   */

  public final static int JOYSTICK_PORT = 1;

  /**
   * CAN Bus IDs for the 4 drive motors
   * 
   * TODO: Update with actual IDs
   */
  public final static int FRONT_LEFT_CAN_ID = 1;
  public final static int FRONT_RIGHT_CAN_ID = 2;
  public final static int BACK_LEFT_CAN_ID = 3;
  public final static int BACK_RIGHT_CAN_ID = 4;

  /**
   * CAN Bus IDs for the 2 flywheel motors
   * 
   * TODO: Update with actual IDs
   */
  public final static int LEFT_FLYWHEEL_MOTOR_CAN_ID = 6;
  public final static int RIGHT_FLYWHEEL_MOTOR_CAN_ID = 5;


  /**
   * Comp can ID 
   */
  public final static int COMPRESSOR_CAN_ID = 0;

  /**
   * Pneumatic Control Module Port IDs for forward and reverse channel of
   * flywheel's double solenoid controlling the feeder piston.
   * 
   * TODO: Update with actual port IDs
   */
  public final static int FLYWHEEL_LEFT_SOLENOID_FORWARD_CHANNEL = 1;
  public final static int FLYWHEEL_LEFT_SOLENOID_REVERSE_CHANNEL = 0;

  
  public final static int FLYWHEEL_RIGHT_SOLENOID_FORWARD_CHANNEL = 2;
  public final static int FLYWHEEL_RIGHT_SOLENOID_REVERSE_CHANNEL = 3;

  /**
   * Strings for the different possible flywheel states.
   */
  public final static String FLYWHEEL_IDLING = "Idling";
  public final static String FLYWHEEL_INTAKING = "Intaking";
  public final static String FLYWHEEL_ACCELERATING = "Accelerating";
  public final static String FLYWHEEL_READY_TO_FIRE = "Ready";
  public final static String FLYWHEEL_FIRING = "Firing";
  public final static String FLYWHEEL_DECELERATING = "Decelerating";

  /**
   * Durations for the different flywheel states
   */

  // TODO: Replace with actual intake duration
  public final static double INTAKE_DURATION_SECONDS = 5.0;
  // TODO: Replace with actual firing duration
  public final static double FIRING_DURATION_SECONDS = 5.0;

  // TODO: Replace with actual intake speed
  public final static double FLYWHEEL_INTAKE_SPEED = -1.0;
  // TODO: Replace with actual Accelerating goal speed
  public final static double FLYWHEEL_FIRING_SPEED = 1.0;
  public final static double FLYWHEEL_FIRING_SPEED_LOW = 0.2;

  // TODO: Replace with actual maximum RPM
  public final static double MAX_FLYWHEEL_RPM = 0;

  public final static double LEFT_FEEDFORWARD_STATIC_VOLTAGE = 0;
  public final static double LEFT_FEEDFORWARD_DRIVING_VOLTAGE = 0;
  public final static double RIGHT_FEEDFORWARD_STATIC_VOLTAGE = 0;
  public final static double RIGHT_FEEDFORWARD_DRIVING_VOLTAGE = 0;


}
