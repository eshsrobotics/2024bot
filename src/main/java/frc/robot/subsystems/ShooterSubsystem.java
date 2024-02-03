// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /**
   * The 2 motor controllers and double solenoid for the flywheel
   */
  private CANSparkMax leftFlywheelMotor;
  private CANSparkMax rightFlywheelMotor;
  private DoubleSolenoid flywheelDoubleSolenoid;

  // private SparkPIDController leftPidController;
  // private SparkPIDController rightPidController;  

  private String currentFlywheelState;
  
  private double stopStateTimer;

  // //Intialize PID controller variables for left flywheel motor
  // private double lP, lI, lD, lIz, lFF, lMinOutput, lMaxOutput;
  // //Intialize PID controller variables for right flywheel motor
  // private double rP, rI, rD, rIz, rFF, rMinOutput, rMaxOutput;

      

  private InputSubsystem inputSubsystem;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(InputSubsystem inputSubsystem) {
    this.inputSubsystem = inputSubsystem;
    leftFlywheelMotor = new CANSparkMax(Constants.LEFT_FLYWHEEL_MOTOR_CAN_ID, CANSparkLowLevel.MotorType.kBrushless);
    rightFlywheelMotor = new CANSparkMax(Constants.RIGHT_FLYWHEEL_MOTOR_CAN_ID, CANSparkLowLevel.MotorType.kBrushless);
    flywheelDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.FLYWHEEL_SOLENOID_FORWARD_CHANNEL, Constants.FLYWHEEL_SOLENOID_REVERSE_CHANNEL);

    // PID Controller Code (VERY UNKNOWN and UNTESTED, uses Spark Pid Controller which has some values I am unfamiliar with. Will do some more research before implementing)
    // leftPidController = leftFlywheelMotor.getPIDController();
    // rightPidController = rightFlywheelMotor.getPIDController();
    // lP = 6e-5; lI = 0; lD = 0; lIz = 0; lFF = 0.000015; lMinOutput = -1; lMaxOutput = 1;
    // rP = 6e-5; rI = 0; rD = 0; rIz = 0; rFF = 0.000015; rMinOutput = -1; rMaxOutput = 1;
    // leftPidController.setP(lP); leftPidController.setI(lI); leftPidController.setD(lD); leftPidController.setIZone(lIz); leftPidController.setFF(lFF); leftPidController.setOutputRange(lMinOutput, lMaxOutput);
    // rightPidController.setP(rP); rightPidController.setI(rI); rightPidController.setD(rD); rightPidController.setIZone(rIz); rightPidController.setFF(rFF); rightPidController.setOutputRange(rMinOutput, rMaxOutput);


    currentFlywheelState = Constants.FLYWHEEL_IDLING;  
    stopStateTimer = 0;

  }

  @Override
  public void periodic() {
   
    // Code for state changing: Checks inputs and changes flywheel state accordingly, if possible
    if (currentFlywheelState.equals(Constants.FLYWHEEL_IDLING) && inputSubsystem.getIntakeButton()) {
        //Update the currentFlywheelState to show we are intaking a disk, set the state timer to Intake's duration, and tell the wheels to spin at the proper intake speed.
        currentFlywheelState = Constants.FLYWHEEL_INTAKING;
        stopStateTimer = Timer.getFPGATimestamp() + Constants.INTAKE_DURATION_SECONDS;
        leftFlywheelMotor.set(Constants.FLYWHEEL_INTAKE_SPEED);
        rightFlywheelMotor.set(Constants.FLYWHEEL_INTAKE_SPEED);
    } 
    else if (currentFlywheelState.equals(Constants.FLYWHEEL_IDLING) && inputSubsystem.getFlywheelAccelerationButton()) {
        //Update the currentFlywheelState to show we are accelerating the flywheel motors
        currentFlywheelState = Constants.FLYWHEEL_ACCELERATING;
    }
    else if (currentFlywheelState.equals(Constants.FLYWHEEL_READY_TO_FIRE) && inputSubsystem.getFiringButton()) {
        //Update the currentFlywheelState to show we are firing a disk
        currentFlywheelState = Constants.FLYWHEEL_FIRING;
        fireFlywheel();
    }
   
    // Code for behavior during states. No input code should happen here

    // If intaking and duration of current state timer has reached the set duration for intaking, move to idling and stop intake motors
    if (stopStateTimer <= Timer.getFPGATimestamp() && stopStateTimer != 0 && currentFlywheelState.equals(Constants.FLYWHEEL_INTAKING)) {
        currentFlywheelState = Constants.FLYWHEEL_IDLING;
        leftFlywheelMotor.stopMotor();
        rightFlywheelMotor.stopMotor();
        stopStateTimer = 0;
    }

     // If Firing and duration of current state timer has reached the set duration for firing, move to decelerating and coast flywheel motors
     if (stopStateTimer <= Timer.getFPGATimestamp() && stopStateTimer != 0 && currentFlywheelState.equals(Constants.FLYWHEEL_FIRING)) {
      currentFlywheelState = Constants.FLYWHEEL_DECELERATING;
      slowFlywheelMotors();
      stopStateTimer = 0;
  }

    // If accelerating or ready to shoot, call the ramp up command and input the designated shooting speed.
    if (currentFlywheelState.equals(Constants.FLYWHEEL_ACCELERATING) || currentFlywheelState.equals(Constants.FLYWHEEL_READY_TO_FIRE)) {
      rampUpFlywheelMotors(Constants.FLYWHEEL_FIRING_SPEED);
    }
    
    // If decelerating, call slow motors command
    if (currentFlywheelState.equals(Constants.FLYWHEEL_DECELERATING)) {
      slowFlywheelMotors();
    }
    if (isFlywheelStopped()) {
      leftFlywheelMotor.stopMotor();
      rightFlywheelMotor.stopMotor();
      currentFlywheelState = Constants.FLYWHEEL_IDLING;
    }

    // If we have reached our shooting speed, change state to notify us that we are ready to launch a disc
    if (isFlywheelReady()) {
      currentFlywheelState = Constants.FLYWHEEL_READY_TO_FIRE;
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  private void rampUpFlywheelMotors(double goalSpeed) {
    //TODO: Write the feedforward loop that will go here
    //WPILib documentation advises not using PID controllers for flywheels, as they are poor at maintaining input voltage once the goal has been reached. https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-flywheel.html
  }

  private void slowFlywheelMotors() {
    //TODO: Write the feedforward loop that will go here
    //Maybe use PID here, as goal is to reach a stop speed, not accelerate and maintain a speed
  }

  private void fireFlywheel() {
    stopStateTimer = Timer.getFPGATimestamp() + Constants.FIRING_DURATION_SECONDS;
    flywheelDoubleSolenoid.set(Value.kForward);

  }

  private boolean isFlywheelReady() {
    //TODO: Tell us if the flywheel has reached it's goal shooting speed, and is ready to fire a disc
    return false;
  }

  private boolean isFlywheelStopped() {
    //TODO: Tell us if the flywheel has stopped spinning and can return to idle
    return false;
  }
}
