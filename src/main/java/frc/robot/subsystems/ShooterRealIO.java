// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.TransportConstants;

public class ShooterRealIO extends Shooter {

  public TalonFX top_motor, bottom_motor;

  public CANSparkMax pitch_motor;

  public ShooterRealIO(){  //FIX PORTS
    top_motor = new TalonFX(0);
    bottom_motor = new TalonFX(0);

    configureTalons();

    pitch_motor = new CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless);

  }

  private void configureTalons(){ //SETUP HELPER FUNCTION
    var configs = new Slot0Configs();

    configs.kP = 0.1; //MOVE TO CONSTANTS
    configs.kV = 0.116;
    configs.kI = 0.0;
    configs.kD = 0.0;
    configs.kS = 0.0;
    configs.kA = 0.0;
    configs.kG = 0.0;
    
    top_motor.getConfigurator().apply(configs);
    bottom_motor.getConfigurator().apply(configs);

    top_motor.setNeutralMode(NeutralModeValue.Coast);
    bottom_motor.setNeutralMode(NeutralModeValue.Coast);
  }

  public void setTopSpeed(double rpm){
    top_motor.setControl(new VelocityVoltage(rpm));
  }

  public void setBottomSpeed(double rpm){
    bottom_motor.setControl(new VelocityVoltage(rpm));
  }

  public void setPitchMotor(double volts){
    pitch_motor.setVoltage(volts);
  }

  public double getPitch(){
    return pitch_motor.getAlternateEncoder(8192).getPosition();
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
