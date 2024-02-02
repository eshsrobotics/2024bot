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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /**
   * The 2 motor controllers and double solenoid for the flywheel
   */
  private CANSparkMax leftFlywheelMotor;
  private CANSparkMax rightFlywheelMotor;
  private DoubleSolenoid flywheelDoubleSolenoid;

  private SparkPIDController leftPidController;
  private SparkPIDController rightPidController;  

  private String currentFlywheelState;
  
  private double stopStateTimer;

  //Intialize PID controller variables for left flywheel motor
  private double lP, lI, lD, lIz, lFF, lMinOutput, lMaxOutput;
  //Intialize PID controller variables for right flywheel motor
  private double rP, rI, rD, rIz, rFF, rMinOutput, rMaxOutput;

      

  private InputSubsystem inputSubsystem;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(InputSubsystem inputSubsystem) {
    this.inputSubsystem = inputSubsystem;
    leftFlywheelMotor = new CANSparkMax(Constants.LEFT_FLYWHEEL_MOTOR_CAN_ID, CANSparkLowLevel.MotorType.kBrushless);
    rightFlywheelMotor = new CANSparkMax(Constants.RIGHT_FLYWHEEL_MOTOR_CAN_ID, CANSparkLowLevel.MotorType.kBrushless);
    flywheelDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.FLYWHEEL_SOLENOID_FORWARD_CHANNEL, Constants.FLYWHEEL_SOLENOID_REVERSE_CHANNEL);

    // PID Controller Code (VERY UNKNOWN and UNTESTED, uses Spark Pid Controller which has some values I am unfamiliar with. Will do some more research before implementing. Member variables are above, I havent commented them out because it would look ugly)
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
        //Update the currentFlywheelState to show we are intaking a disk
        currentFlywheelState = Constants.FLYWHEEL_INTAKING;
        stopStateTimer = Timer.getFPGATimestamp() + Constants.INTAKE_DURATION_SECONDS;
    } 
    else if (currentFlywheelState.equals(Constants.FLYWHEEL_IDLING) && inputSubsystem.getFlywheelAccelerationButton()) {
        //Update the currentFlywheelState to show we are accelerating the flywheel motors
        currentFlywheelState = Constants.FLYWHEEL_ACCELERATING;
    }
    else if (currentFlywheelState.equals(Constants.FLYWHEEL_READY_TO_FIRE) && inputSubsystem.getFiringButton()) {
        //Update the currentFlywheelState to show we are firing a disk
        currentFlywheelState = Constants.FLYWHEEL_ACCELERATING;
    }
   
    // Code for behavior during states. No input code should happen here
    if (currentFlywheelState.equals(Constants.FLYWHEEL_INTAKING)) {
        leftFlywheelMotor.set(Constants.FLYWHEEL_INTAKE_SPEED);
        rightFlywheelMotor.set(Constants.FLYWHEEL_INTAKE_SPEED);
    }

    if (stopStateTimer <= Timer.getFPGATimestamp() && (currentFlywheelState.equals(Constants.FLYWHEEL_INTAKING))) {
        currentFlywheelState = Constants.FLYWHEEL_IDLING;
        leftFlywheelMotor.set(0);
        rightFlywheelMotor.set(0);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  private void rampUpFlywheelMotors(double goalSpeed) {

  }
}
