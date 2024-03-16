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

  }

  @Override
  public void periodic() {

    switch (currentFlywheelState) {

      case Constants.FLYWHEEL_IDLING:
        if (inputSubsystem.getFiringButton()) {
          flywheelDoubleSolenoidLeft.set(Value.kForward);
          flywheelDoubleSolenoidRight.set(Value.kForward);
          currentFlywheelState = Constants.FLYWHEEL_INTAKING;
          stopStateTimer = Timer.getFPGATimestamp() + Constants.INTAKE_DURATION_SECONDS;
          leftFlywheelMotor.set(-Constants.FLYWHEEL_INTAKE_SPEED);
          rightFlywheelMotor.set(-Constants.FLYWHEEL_INTAKE_SPEED);
        } else if (inputSubsystem.getIntakeButton()) {
          flywheelDoubleSolenoidLeft.set(Value.kReverse);
          flywheelDoubleSolenoidRight.set(Value.kReverse);
          currentFlywheelState = Constants.FLYWHEEL_INTAKING;
          stopStateTimer = Timer.getFPGATimestamp() + Constants.INTAKE_DURATION_SECONDS;
          leftFlywheelMotor.set(Constants.FLYWHEEL_INTAKE_SPEED);
          rightFlywheelMotor.set(Constants.FLYWHEEL_INTAKE_SPEED);
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

      case Constants.FLYWHEEL_ACCELERATING:
        if (stopStateTimer < Timer.getFPGATimestamp()) {
          currentFlywheelState = Constants.FLYWHEEL_IDLING;
          leftFlywheelMotor.stopMotor();
          rightFlywheelMotor.stopMotor();
          stopStateTimer = 0;
        }
        break;

      case Constants.FLYWHEEL_READY_TO_FIRE:
        rampUpFlywheelMotors(Constants.FLYWHEEL_FIRING_SPEED);
        if (inputSubsystem.getFiringButton()) {
          currentFlywheelState = Constants.FLYWHEEL_FIRING;
        }
        break;

      case Constants.FLYWHEEL_FIRING:
        fireFlywheel();
        flywheelDoubleSolenoidLeft.set(Value.kForward);
        flywheelDoubleSolenoidRight.set(Value.kForward);
        if (stopStateTimer < Timer.getFPGATimestamp()) {
          currentFlywheelState = Constants.FLYWHEEL_DECELERATING;
          slowFlywheelMotors();
          stopStateTimer = 0;
          flywheelDoubleSolenoidLeft.set(Value.kReverse);
          flywheelDoubleSolenoidRight.set(Value.kReverse);
        }
        break;

      case Constants.FLYWHEEL_DECELERATING:
        if (slowFlywheelMotors()) {
          currentFlywheelState = Constants.FLYWHEEL_IDLING;
        }
        break;
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Used to accelerate flywheel motors to desired speed, and to maintain the
   * maximum speed
   * 
   * @param goalSpeed
   */
  private void rampUpFlywheelMotors(double goalSpeed) {
    double leftVelocity = leftFlywheelEncoder.getVelocity();
    double rightVelocity = rightFlywheelEncoder.getVelocity();
    leftFlywheelMotor.set(leftMotorFeedforward.calculate(leftVelocity));
    rightFlywheelMotor.set(rightMotorFeedforward.calculate(rightVelocity));
  }

  /**
   * Called after firing to safely decelerate flywheel
   * 
   * @return returns if flywheel has reached a full stop
   */
  private boolean slowFlywheelMotors() {
    return false;
    // TODO: Write the feedforward loop that will go here
    // Maybe use PID here, as goal is to reach a stop speed, not accelerate and
    // maintain a speed
  }

  /**
   * Called when told to fire. Maintains current flywheel speed,
   * 
   * @return returns if flywheel has reached a full stop
   */
  private void fireFlywheel() {
    rampUpFlywheelMotors(Constants.FLYWHEEL_FIRING_SPEED);
    if (stopStateTimer == 0) {
      stopStateTimer = Timer.getFPGATimestamp() + Constants.FIRING_DURATION_SECONDS;
    }
  }

  private boolean isFlywheelReady() {
    // TODO: Tell us if the flywheel has reached it's goal shooting speed, and is
    // ready to fire a disc
    return false;
  }
}
