// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ClimberConstants;

public class ClimberRealIO extends Climber {

  public static ClimberRealIO m_instance;

  public DigitalInput l_switch, r_switch;

  public CANSparkMax l_motor, r_motor;

  public ClimberRealIO() {
    l_switch = new DigitalInput(0);
    r_switch = new DigitalInput(1);
    l_motor = new CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless); //ADD PORTS
    r_motor = new CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless);
  }

  public void setLeftMotor(double speed){
    l_motor.setVoltage(speed);
  }

  public void setRightMotor(double speed){
    r_motor.setVoltage(speed);
  }

  public boolean getLeftSwitch(){
    return l_switch.get();
  }

  public boolean getRightSwitch(){
    return r_switch.get();
  }

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
