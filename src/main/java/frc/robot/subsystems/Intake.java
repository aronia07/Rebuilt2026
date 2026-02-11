// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeWantedState;
import frc.robot.Constants.IntakeConstants.SystemState;
import frc.util.LoggedTunableNumber;

public class Intake extends SubsystemBase {
  /* MOTORS */
  private TalonFX intakeMotor = new TalonFX(IntakeConstants.intakeMotorID, "rio");
  private TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();
  //for velocity control
  private double motorspeed = 0.0;
  final MotionMagicVelocityVoltage mm_request = new MotionMagicVelocityVoltage(0);
  //for position control
  private double position = 0.0;
  final MotionMagicExpoVoltage mmE_request = new MotionMagicExpoVoltage(0);
  final DutyCycleOut m_leftrequestOut = new DutyCycleOut(0.0);
  /* STATES */
  IntakeWantedState wantedState = IntakeWantedState.IDLE;
  SystemState systemState = SystemState.IDLING;
  IntakeWantedState wantedState2 = IntakeWantedState.INTAKE;
  SystemState systemState2 = SystemState.INTAKING;
  IntakeWantedState wantedState3 = IntakeWantedState.SHOOT;
  SystemState systemState3 = SystemState.SHOOTING;
  IntakeWantedState wantedState4 = IntakeWantedState.RETRACT;
  SystemState systemState4 = SystemState.RETRACTING;
  

  /** Creates a new Intake */
  public Intake() {
    /* SETUP CONFIG */
    
    // CURRENT LIMITS
    intakeMotorConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SupplyCurrentLimit;
    intakeMotorConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.StatorCurrentLimit;

    //APPLY CONFIG TO MOTOR
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = intakeMotor.getConfigurator().apply(intakeMotorConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  public void setWantedIntakeMode(IntakeWantedState desiredState) {
    this.wantedState = desiredState;
  }

  private SystemState changeCurrentSystemState() {
    return switch (wantedState) {
      case IDLE: 
        yield SystemState.IDLING;
      case SHOOT :
        yield SystemState.SHOOTING;
      case INTAKE :
        yield SystemState.INTAKING;
      case RETRACT :
        yield SystemState.RETRACTING;
    };
  }

    
  private void applyState(){
    switch(systemState){
      case IDLING:
        motorspeed = 0.0;
        position = 0.0;
        break;
      case INTAKING:
        position = IntakeConstants.intakingPosition; 
        motorspeed = IntakeConstants.intakingSpeed;
        break;
      case SHOOTING :
        motorspeed = IntakeConstants.shootingPosition;
        break;
      case RETRACTING :
        motorspeed = IntakeConstants.retractingPos;

    }
  }
  
  /**
   * Check LoggedTunableNumbers. If changed, update PID and SVA values of motor
   */
  // public void checkTunableValues() {
  //   if (k_S.hasChanged() || k_V.hasChanged() || k_A.hasChanged() 
  //   || k_P.hasChanged() || k_I.hasChanged() || k_D.hasChanged()) {
  //     intakeMotorConfig.Slot0.kS = k_S.get();
  //     intakeMotorConfig.Slot0.kV = k_V.get();
  //     intakeMotorConfig.Slot0.kA = k_A.get(); 
  //     intakeMotorConfig.Slot0.kP = k_P.get();
  //     intakeMotorConfig.Slot0.kI = k_I.get();
  //     intakeMotorConfig.Slot0.kD = k_D.get();
  //   }
  //   intakeMotor.getConfigurator().apply(intakeMotorConfig);
  // }

  @Override
  public void periodic() {
    // checkTunableValues();
    systemState = changeCurrentSystemState();
    applyState();
    //example of how to control motor for velocity
    intakeMotor.setControl(mm_request.withVelocity(motorspeed));
    //example of how to control motor for position
    intakeMotor.setControl(mmE_request.withPosition(position));
    // setting the request to the motor controller
    intakeMotor.setControl(m_leftrequestOut.withOutput(motorspeed));
  }

}
 