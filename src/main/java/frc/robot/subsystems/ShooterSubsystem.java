// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  // The 2 motor controllers and motor encoders for the flywheel
  private CANSparkMax leftFlywheelMotor;
  private CANSparkMax rightFlywheelMotor;
  private RelativeEncoder leftFlywheelEncoder;
  private RelativeEncoder rightFlywheelEncoder;

  // Solenoid for pneumatic piston feeder in shooting system
  private DoubleSolenoid flywheelDoubleSolenoidLeft;
  private DoubleSolenoid flywheelDoubleSolenoidRight;

  private Compressor compressor;

  // String tracking our current state
  private String currentFlywheelState;

  // Timer to track durations of states with set runtimes
  private double stopStateTimer;

  // When we transition from the FLYWHEEL_IDLING to FLYWHEEL_INTAKING
  // state, this variable remembers the speed we originally set
  // (it could range from -1.0, meaning shooting full forward, to +1.0,
  // meaning itaking full backward.)
  private double originalIntakingSpeed;

  // Feedforward controllers to manage speeds of flywheels & their respective
  // values
  private double leftKs, leftKv, rightKs, rightKv;
  private SimpleMotorFeedforward leftMotorFeedforward;
  private SimpleMotorFeedforward rightMotorFeedforward;

  private InputSubsystem inputSubsystem;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(InputSubsystem inputSubsystem) {

    
    compressor = new Compressor(Constants.COMPRESSOR_CAN_ID, PneumaticsModuleType.CTREPCM);
    
    this.inputSubsystem = inputSubsystem;
    leftFlywheelMotor = new CANSparkMax(Constants.LEFT_FLYWHEEL_MOTOR_CAN_ID, CANSparkLowLevel.MotorType.kBrushless);
    rightFlywheelMotor = new CANSparkMax(Constants.RIGHT_FLYWHEEL_MOTOR_CAN_ID, CANSparkLowLevel.MotorType.kBrushless);
    rightFlywheelMotor.setInverted(true);
    flywheelDoubleSolenoidLeft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.FLYWHEEL_LEFT_SOLENOID_FORWARD_CHANNEL,
        Constants.FLYWHEEL_LEFT_SOLENOID_REVERSE_CHANNEL);
    flywheelDoubleSolenoidRight = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.FLYWHEEL_RIGHT_SOLENOID_FORWARD_CHANNEL,
        Constants.FLYWHEEL_RIGHT_SOLENOID_REVERSE_CHANNEL);

    flywheelDoubleSolenoidLeft.set(Value.kReverse);
    flywheelDoubleSolenoidRight.set(Value.kReverse);

    leftFlywheelEncoder = leftFlywheelMotor.getEncoder();
    rightFlywheelEncoder = rightFlywheelMotor.getEncoder();

    leftKs = Constants.LEFT_FEEDFORWARD_STATIC_VOLTAGE;
    leftKv = Constants.LEFT_FEEDFORWARD_DRIVING_VOLTAGE;
    rightKs = Constants.RIGHT_FEEDFORWARD_STATIC_VOLTAGE;
    rightKv = Constants.RIGHT_FEEDFORWARD_DRIVING_VOLTAGE;

    leftMotorFeedforward = new SimpleMotorFeedforward(leftKs, leftKv);
    rightMotorFeedforward = new SimpleMotorFeedforward(rightKs, rightKv);

    currentFlywheelState = Constants.FLYWHEEL_IDLING;
    stopStateTimer = 0;
    originalIntakingSpeed = 0;

  }

  @Override
  public void periodic() {

    switch (currentFlywheelState) {

      case Constants.FLYWHEEL_IDLING:
        if (inputSubsystem.getLowGoalFiringButton()) {
          flywheelDoubleSolenoidLeft.set(Value.kForward);
          flywheelDoubleSolenoidRight.set(Value.kForward);
          currentFlywheelState = Constants.FLYWHEEL_INTAKING;
          stopStateTimer = Timer.getFPGATimestamp() + Constants.INTAKE_DURATION_SECONDS;
          originalIntakingSpeed = Constants.FLYWHEEL_FIRING_SPEED_LOW;
          leftFlywheelMotor.set(originalIntakingSpeed);
          rightFlywheelMotor.set(originalIntakingSpeed);
        } else if (inputSubsystem.getHighGoalFiringButton()) {
          flywheelDoubleSolenoidLeft.set(Value.kForward);
          flywheelDoubleSolenoidRight.set(Value.kForward);
          currentFlywheelState = Constants.FLYWHEEL_INTAKING;
          stopStateTimer = Timer.getFPGATimestamp() + Constants.INTAKE_DURATION_SECONDS;
          originalIntakingSpeed = Constants.FLYWHEEL_FIRING_SPEED;
          leftFlywheelMotor.set(originalIntakingSpeed);
          rightFlywheelMotor.set(originalIntakingSpeed);
        } else if (inputSubsystem.getIntakeButton()) {
          flywheelDoubleSolenoidLeft.set(Value.kReverse);
          flywheelDoubleSolenoidRight.set(Value.kReverse);
          currentFlywheelState = Constants.FLYWHEEL_INTAKING;
          stopStateTimer = Timer.getFPGATimestamp() + Constants.INTAKE_DURATION_SECONDS;
          originalIntakingSpeed = Constants.FLYWHEEL_INTAKE_SPEED;
          leftFlywheelMotor.set(originalIntakingSpeed);
          rightFlywheelMotor.set(originalIntakingSpeed);
        }
        break;

      case Constants.FLYWHEEL_INTAKING:
        if (stopStateTimer < Timer.getFPGATimestamp()) {
          currentFlywheelState = Constants.FLYWHEEL_IDLING;
          leftFlywheelMotor.stopMotor();
          rightFlywheelMotor.stopMotor();
          stopStateTimer = 0;
        }
        break;

    }
  }


  public void fire(double firingSpeed) {

    if (firingSpeed > 0) {
      flywheelDoubleSolenoidLeft.set(Value.kForward);
      flywheelDoubleSolenoidRight.set(Value.kForward);
    } else {
      flywheelDoubleSolenoidLeft.set(Value.kReverse);
      flywheelDoubleSolenoidRight.set(Value.kReverse);
    }
    originalIntakingSpeed = firingSpeed;
    leftFlywheelMotor.set(originalIntakingSpeed);
    rightFlywheelMotor.set(originalIntakingSpeed);

    stopStateTimer = Timer.getFPGATimestamp() + Constants.INTAKE_DURATION_SECONDS;

    currentFlywheelState = Constants.FLYWHEEL_INTAKING;
  }


}
