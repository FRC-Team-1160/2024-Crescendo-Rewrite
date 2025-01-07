// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.DriveMotorConfigs;
import frc.robot.Constants.SwerveConstants.SteerMotorConfigs;


public class SwerveModuleRealIO extends SwerveModule{

  public TalonFX steer_motor, drive_motor;

  public CANcoder steer_sensor;

  public SwerveModuleRealIO(int drive_port, int steer_port, int sensor_port){
    drive_motor = new TalonFX(drive_port);
    steer_motor = new TalonFX(steer_port);
    steer_sensor = new CANcoder(sensor_port);

    Slot0Configs driveConfigs = new Slot0Configs();
    driveConfigs.kV = DriveMotorConfigs.kV;
    driveConfigs.kP = DriveMotorConfigs.kP;
    driveConfigs.kI = DriveMotorConfigs.kI;
    driveConfigs.kD = DriveMotorConfigs.kD;
    drive_motor.getConfigurator().apply(driveConfigs);

    Slot0Configs steerConfigs = new Slot0Configs();
    steerConfigs.kV = SteerMotorConfigs.kV;
    steerConfigs.kP = SteerMotorConfigs.kP;
    steerConfigs.kI = SteerMotorConfigs.kI;
    steerConfigs.kD = SteerMotorConfigs.kD;
    steer_motor.getConfigurator().apply(steerConfigs);

  }

  public double getSpeed(){
    return drive_motor.getRotorVelocity().getValue() * SwerveConstants.WHEEL_ROTOR_TO_METERS;
  }

  public double getPosition(){
    return drive_motor.getRotorPosition().getValue() * SwerveConstants.WHEEL_ROTOR_TO_METERS;
  }

  public Rotation2d getAngle(){
    double a = steer_sensor.getAbsolutePosition().getValue();
    //wrap from -pi to pi radians
    a = MathUtil.inputModulus(a, -0.5, 0.5);
    
    return Rotation2d.fromRotations(a);
  }

  public SwerveModuleState getModuleState(){
    return new SwerveModuleState(getSpeed(), getAngle());
  }

  public SwerveModulePosition getModulePosition(){
    return new SwerveModulePosition(getPosition(), getAngle());
  }

  public void setDrive(double speedMetersPerSecond){
    SmartDashboard.putNumber("in_speed", speedMetersPerSecond / SwerveConstants.WHEEL_ROTOR_TO_METERS);
    drive_motor.setControl(new VelocityVoltage(speedMetersPerSecond / SwerveConstants.WHEEL_ROTOR_TO_METERS));
  }

  public void setSteer(double volts){
    steer_motor.setVoltage(volts);
  }

}
