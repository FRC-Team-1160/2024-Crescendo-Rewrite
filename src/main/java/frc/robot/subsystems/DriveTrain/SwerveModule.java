// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;


public abstract class SwerveModule {

  public SwerveModuleState target_state;

  public PIDController angle_pid;

  public SwerveModule(){

    target_state = new SwerveModuleState();

  }

  public void setState(SwerveModuleState state){
    target_state = state;
    setDrive(state.speedMetersPerSecond);
  }

  public void update(){
    double target_angle = target_state.angle.getRotations();
    double a = angle_pid.calculate(getAngle().getRotations(), target_angle);
    if (Math.abs(getAngle().getRotations() - target_angle) > 0.005){
        setSteer(-a);
    } else {
        setSteer(0);
    }
  }

  abstract double getSpeed();

  abstract double getPosition();

  abstract Rotation2d getAngle();

  abstract SwerveModuleState getModuleState();

  abstract SwerveModulePosition getModulePosition();

  abstract void setDrive(double speedMetersPerSecond);

  abstract void setSteer(double volts);
}
