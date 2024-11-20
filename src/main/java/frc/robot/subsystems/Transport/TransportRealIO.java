// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Transport;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ClimberConstants;

public class TransportRealIO extends Transport {

  public CANSparkMax l_wheel, r_wheel, m_belt;

  public ColorSensorV3 m_color_sensor;

  public DigitalInput m_limit;

  /** Creates a new ExampleSubsystem. */
  public TransportRealIO() { //FILL IN ID CONSTANTS
    l_wheel = new CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless);
    r_wheel = new CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless);
    m_belt = new CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless);
  }

  public void setWheels(double volts){
    l_wheel.setVoltage(-volts);
    r_wheel.setVoltage(volts);
  }

  public void setBelt(double volts){
    m_belt.setVoltage(volts);
  }

  public boolean getSwitchPressed(){
    return !m_limit.get();
  }

  public double readColorProx(){
    prox = m_color_sensor.getProximity();
    return prox;
  }

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
