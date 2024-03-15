// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  /**
   * The 4 motor controllers for the drive
   */
  private CANSparkMax frontLeft;
  private CANSparkMax frontRight;
  private CANSparkMax backLeft;
  private CANSparkMax backRight;
  private PWMMotorController testMotor;

  /**
   * To make the robot drive, we must use the Mecanum formula to calculate all
   * four wheel speeds.
   * WPILib has a class to do this.
   * 
   * This class is _only_ used to drive during teleop. During autonomous, we use a
   * {@link HolonomicDriveController} instead.
   */
  private MecanumDrive drive;

  /**
   * {@link MecanumDriveKinematics} converts {@link ChassisSpeeds} to a
   * {@link MecanumDriveWheelSpeeds}
   */
  private MecanumDriveKinematics kinematics;

  /**
   * Keeps track of the bot's position in space.
   */
  private MecanumDriveOdometry mecanumDriveOdometry;

  /**
     * Keeps track of the robot's rotation since the last call to {@link #reset()}.
     */
    private ADXRS450_Gyro gyro;
    private InputSubsystem inputSubsystem;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(InputSubsystem inputSubsystem) {
      System.out.println("Drive is initialized");
    this.inputSubsystem = inputSubsystem;
    frontLeft = new CANSparkMax(Constants.FRONT_LEFT_CAN_ID, CANSparkLowLevel.MotorType.kBrushed);
    frontRight = new CANSparkMax(Constants.FRONT_RIGHT_CAN_ID, CANSparkLowLevel.MotorType.kBrushed);
    backLeft = new CANSparkMax(Constants.BACK_LEFT_CAN_ID, CANSparkLowLevel.MotorType.kBrushed);
    backRight = new CANSparkMax(Constants.BACK_RIGHT_CAN_ID, CANSparkLowLevel.MotorType.kBrushed);
    frontLeft.setInverted(true);
    backLeft.setInverted(true);
    backRight.setInverted(false);
    testMotor = new Spark(0);
    drive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
    gyro = new ADXRS450_Gyro();
  }

  @Override
  public void periodic() {
    
    System.out.println(inputSubsystem.getFrontBack());
    // testMotor.set(inputSubsystem.getFrontBack());
    drive.driveCartesian(inputSubsystem.getFrontBack(), inputSubsystem.getLeftRight(), inputSubsystem.getRotation(), gyro.getRotation2d());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
