// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Scoring;

import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightsConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.TurretConstants.TurretWantedState;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Lights.LEDSubsystem_WPIlib;
import frc.robot.Constants.TurretConstants.SystemState;
import frc.util.LoggedTunableNumber;

public class Turret extends SubsystemBase {
  private final CommandSwerveDrivetrain drivetrain;
  private final LEDSubsystem_WPIlib leds;
  /* MOTORS */
  private TalonFX turretMotor = new TalonFX(TurretConstants.turretMotorID, "rio");
  private TalonFXConfiguration turretMotorConfig = new TalonFXConfiguration();
  
  /* ENCODERS */
  private CANcoder encoder = new CANcoder(TurretConstants.encoderID, "rio");

  //for position control
  private double position = 0.0;
  private double CCWlimit = 0.85;
  private double CWLimit = -0.85;

  final PositionVoltage mmE_request = new PositionVoltage(0);

  /* PIDFF CONTROL */
  private LoggedTunableNumber k_S = new LoggedTunableNumber("turret_s", TurretConstants.turretSVA[0]);
  private LoggedTunableNumber k_V = new LoggedTunableNumber("turret_v", TurretConstants.turretSVA[1]);
  private LoggedTunableNumber k_A = new LoggedTunableNumber("turret_a", TurretConstants.turretSVA[2]);

  private LoggedTunableNumber k_P = new LoggedTunableNumber("turret_p", TurretConstants.turretPID[0]);
  private LoggedTunableNumber k_I = new LoggedTunableNumber("turret_i", TurretConstants.turretPID[1]);
  private LoggedTunableNumber k_D = new LoggedTunableNumber("turret_d", TurretConstants.turretPID[2]);

  /* STATES */
  TurretWantedState wantedState = TurretWantedState.IDLE;
  SystemState systemState = SystemState.IDLING;


  /** Creates a new Turret */
  public Turret(CommandSwerveDrivetrain drivetrain, LEDSubsystem_WPIlib leds) {
    this.drivetrain = drivetrain;
    this.leds = leds;

    /* SETUP CONFIG */
    
    // CURRENT LIMITS
    turretMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turretMotorConfig.CurrentLimits.SupplyCurrentLimit = TurretConstants.SupplyCurrentLimit;
    turretMotorConfig.CurrentLimits.StatorCurrentLimit = TurretConstants.StatorCurrentLimit;

    turretMotorConfig.Feedback.FeedbackRemoteSensorID = 54;
    turretMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    turretMotorConfig.ClosedLoopGeneral.ContinuousWrap = false;
    
    //PID CONSTANTS
    turretMotorConfig.Slot0.kS = k_S.get();
    turretMotorConfig.Slot0.kV = k_V.get();
    turretMotorConfig.Slot0.kA = k_A.get(); 
    turretMotorConfig.Slot0.kP = k_P.get();
    turretMotorConfig.Slot0.kI = k_I.get();
    turretMotorConfig.Slot0.kD = k_D.get();

    //use this for velocity motion magic 
    turretMotorConfig.MotionMagic.MotionMagicAcceleration = TurretConstants.turretMotionMagicAccel; // Target acceleration of 160 rps/s (0.5 seconds)
    turretMotorConfig.MotionMagic.MotionMagicJerk = TurretConstants.turretMotionMagicJerk;

    //use this for motion magic expo (very good control of position)
    turretMotorConfig.MotionMagic.MotionMagicExpo_kV = TurretConstants.turretMotionMagicExpoK_V;
    turretMotorConfig.MotionMagic.MotionMagicExpo_kA = TurretConstants.turretMotionMagicExpoK_A;

    //APPLY CONFIG TO MOTOR
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = turretMotor.getConfigurator().apply(turretMotorConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  public void setWantedTurretState(TurretWantedState desiredState) {
    this.wantedState = desiredState;
  }

  private SystemState changeCurrentSystemState() {
    return switch (wantedState) {
      case IDLE: 
        yield SystemState.IDLING;
      case AIM:
        if (drivetrain.isInAllianceZone()) {
          yield SystemState.HUB_AIMING;
        } else {
          yield SystemState.PASS_AIMING;
        }
      case TRENCH_PRESET:
        yield SystemState.TRENCH_PRESETTING;
      case CLOSE_PRESET:
        yield SystemState.CLOSE_PRESETTING;
      case TEST:
        yield SystemState.TESTING;
        
    };
  }
  

    
  private void applyState(){
    switch(systemState){
      case IDLING:
        position = 0.0;
        break;
      case PASS_AIMING:
        leds.LED_SolidColor(LightsConstants.RBGColors.get("magenta"));
        position = TurretConstants.passAimPosition;
        break;
      case HUB_AIMING:
        leds.LED_SolidColor(LightsConstants.RBGColors.get("yellow"));
        double calcTurretAngle = 
          (-drivetrain.getTurretPose().getRotation().getDegrees() + Units.radiansToDegrees(Math.atan(drivetrain.getYfromHub() / drivetrain.getXfromHub())))/360;
          SmartDashboard.putNumber("Calculated Turret setpint", calcTurretAngle);
        if(calcTurretAngle < CWLimit) {
          calcTurretAngle = calcTurretAngle + 1;
        } if (calcTurretAngle > CCWlimit) {
          calcTurretAngle = calcTurretAngle - 1;
        }
        SmartDashboard.putNumber("Turret Setpoint with adjustment", calcTurretAngle);

        position = calcTurretAngle;
        //convert to rotations, set limits, 
        break;
      case TRENCH_PRESETTING:
        position = TurretConstants.trenchPresetPosition;
        break;
      case TESTING:
        position = .75;
        break;
    }
  }  


  /**
   * Check LoggedTunableNumbers. If changed, update PID and SVA values of motor
   */
  public void checkTunableValues() {
    if (k_S.hasChanged() || k_V.hasChanged() || k_A.hasChanged() 
    || k_P.hasChanged() || k_I.hasChanged() || k_D.hasChanged()) {
      turretMotorConfig.Slot0.kS = k_S.get();
      turretMotorConfig.Slot0.kV = k_V.get();
      turretMotorConfig.Slot0.kA = k_A.get(); 
      turretMotorConfig.Slot0.kP = k_P.get();
      turretMotorConfig.Slot0.kI = k_I.get();
      turretMotorConfig.Slot0.kD = k_D.get();

      turretMotor.getConfigurator().apply(turretMotorConfig);

    }
  }

  private void logValues() {
    SmartDashboard.putNumber("Turret Absolute Position", encoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("Turret Motor Position", turretMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Turret Wanted Position", position);
    SmartDashboard.putNumber("Turret Encoder Position", encoder.getPosition().getValueAsDouble());
    SmartDashboard.putString("TURRET WANTED STATE", wantedState.toString());
    SmartDashboard.putString("TURRET SYSTEM STATE", systemState.toString());
  }

  @Override
  public void periodic() {
    logValues();
    systemState = changeCurrentSystemState();
    applyState();
    //example of how to control motor for position
    turretMotor.setControl(mmE_request.withPosition(position));
  }

}
