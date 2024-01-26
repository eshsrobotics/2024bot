// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.SPI.Port;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  
        /**
     * If an Xbox controller is plugged in, we assume it is in this port
     * 
     * <p>WARNING: If using an Xbox controller and a joystick at the same time, 
     * plug the Xbox controller in first</p>
     */
    public final static int XBOX_PORT = 0;

    /**
     * If a joystick is plugged in, we assume it is in this port
     * 
     * <p>WARNING: If using an Xbox controller and a joystick at the same time, 
     * plug the joystick in second</p>
     */

    public final static int JOYSTICK_PORT = 1;

    /**
     * TODO: update with real values
     * CAN Bus IDs for the 4 drive motors
     */
    public final static int FRONT_LEFT_CAN_ID = 1;
    public final static int FRONT_RIGHT_CAN_ID = 1;
    public final static int BACK_LEFT_CAN_ID = 1;
    public final static int BACK_RIGHT_CAN_ID = 1;
   

}
