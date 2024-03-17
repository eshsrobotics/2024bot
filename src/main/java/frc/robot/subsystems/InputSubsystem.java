package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** 
 * Abstracts user input to three different values for, the front-back value, the
 * left-right value, and the rotation value for a mecanum drive
 */
public class InputSubsystem extends SubsystemBase {
    /**
     * The XboxController object, will be used if our controls will be from an
     * Xbox controller
     */
    private XboxController xboxController = null;

    /**
     * The Joystick object, will be used if our controls will be from a joystick
     */
    private Joystick joystickController = null;

    private double frontBack;
    private double leftRight;
    private double rotation;

    private boolean autonRunning;
    private boolean lowGoalFireButtonIsDepressed;
    private boolean highGoalFireButtonIsDepressed;
    private boolean intakeButtonIsDepressed;
    private boolean accelerateFlywheelButtonIsDepressed;

    public InputSubsystem() {
        autonRunning = false;
        try {
            joystickController = new Joystick(Constants.JOYSTICK_PORT); 
        } catch(Exception e) {
            System.out.format("Exception caught while initializing input subsystem: %s\n", e.getMessage());
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        
        lowGoalFireButtonIsDepressed = false;
        intakeButtonIsDepressed = false;
        accelerateFlywheelButtonIsDepressed = false;

        double xboxFrontBack = 0.0;
        double xboxLeftRight = 0.0;
        double xboxRotation = 0.0;
        double joystickFrontBack = 0.0;
        double joystickLeftRight = 0.0;
        double joystickRotation = 0.0;
        
        if (joystickController != null) {
            
            lowGoalFireButtonIsDepressed = joystickController.getRawButtonPressed(6);
            highGoalFireButtonIsDepressed = joystickController.getRawButtonPressed(1);

            //TODO: Update with driver's preferred intake button
            intakeButtonIsDepressed = joystickController.getRawButtonPressed(5);

            //TODO: Update with driver's preferred flywheel acceleration button
            accelerateFlywheelButtonIsDepressed = joystickController.getRawButtonPressed(6);

            // Apparently the joystick that we were competing with did not give
            // consistent results when the Y-axis was zeroed out. We added a
            // kludge to overcome this (which is not the same as a deadzone,
            // though maybe we should using one of those instead) and ran into
            // the problem of our kludge affecting the drive during auton!
            //
            // InputSubsystem.periodic() always runs, whether auton is running
            // or not, but we only want to employ the kludge here if we are
            // fully in teleop.
            double joystickKludge = 0.15;
            if (autonRunning) {
                joystickKludge = 0;
            }
            joystickFrontBack = -joystickController.getY() + joystickKludge;          

            joystickLeftRight = joystickController.getX();
            if (joystickController.getRawButton(9)) {
                joystickRotation = -0.3;
            } else if (joystickController.getRawButton(10)) {
                joystickRotation = 0.3;
            } else if (joystickController.getRawButton(11)) {
                joystickRotation = -0.7;
            } else if (joystickController.getRawButton(12)) {
                joystickRotation = 0.7;
            }
        }
        System.out.println(joystickRotation);

        // Intelligently combine simultaneous inputs
        frontBack = MathUtil.clamp(joystickFrontBack, -1, 1);
        leftRight = MathUtil.clamp(joystickLeftRight, -1, 1);
        rotation = MathUtil.clamp(joystickRotation, -1, 1);
        if (Math.abs(frontBack) <= 0.05) {
            frontBack = 0;
        }                                                                                                                                           
        if (Math.abs(leftRight) <= 0.05) {
            leftRight = 0;
        }
        if (Math.abs(rotation) <= 0.05) {
            rotation = 0;
        }
    }

    /**
     * The user's desired front-back motion
     */
    public double getFrontBack() {
        return frontBack;
    }

    /**
     * The user's desired left-right motion
     */
    public double getLeftRight() {
        return leftRight;
    }

    /**
     * The user's desired rotational motion
     */
    public double getRotation() {
        return rotation;
    }

    /**
     * Returns whether the firing button was pressed
     */
    public boolean getLowGoalFiringButton() {
        return lowGoalFireButtonIsDepressed;
    }

    public boolean getHighGoalFiringButton() {
        return highGoalFireButtonIsDepressed;
    }

    /**
     * Returns whether the flywheel acceleration button was pressed
     */
    public boolean getFlywheelAccelerationButton() {
        return accelerateFlywheelButtonIsDepressed;
    }


    /**
     * Returns whether the intake button was pressed
     */
    public boolean getIntakeButton() {
        return intakeButtonIsDepressed;
    }

    public void setAutonState(boolean isAutonRunning) {
        autonRunning = isAutonRunning;
    }

}