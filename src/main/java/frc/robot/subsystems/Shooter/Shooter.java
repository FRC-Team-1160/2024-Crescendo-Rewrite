// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.TransportConstants;

public abstract class Shooter extends SubsystemBase {

  protected PIDController pitch_pid;

  public Shooter(){
    pitch_pid = new PIDController(3.5, 50.0, 0); //MOVE TO CONSTANTS
    pitch_pid.setIntegratorRange(-0.005, 0.005);
    pitch_pid.setIZone(0.02);
    pitch_pid.setSetpoint(getPitch());
  }

  /**
   * Gets the setpoint of the pitch PID controller.
   * @return The setpoint of the pitch PID controller.
   */
  public double getPitchSetpoint(){
    return pitch_pid.getSetpoint();
  }

  /**
   * Sets the setpoint of the pitch PID controller.
   * @param setpoint The desired setpoint
   */
  public void setPitchSetpoint(double setpoint){
    pitch_pid.setSetpoint(setpoint);
  }

  /**
   * Sets the speed of both flywheel motors.
   * @param rpm The desired speed in rpm.
   */
  public void setSpeed(double rpm){
    setTopSpeed(rpm);
    setBottomSpeed(rpm);
  }

  abstract void setTopSpeed(double rpm);

  abstract void setBottomSpeed(double rpm);

  abstract void setPitchMotor(double volts);

  /**
   * Gets the measured pitch from the encoder.
   * @return The measured pitch.
   */
  abstract double getPitch();

  @Override
  public void periodic() {
    setPitchMotor(pitch_pid.calculate(getPitch()));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
