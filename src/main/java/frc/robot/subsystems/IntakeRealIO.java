// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.Ports;

public class IntakeRealIO extends Intake {

  private DoubleSolenoid m_solenoid;
  
  public CANSparkMax m_feed_motor;  


  /** Creates a new ExampleSubsystem. */
  public IntakeRealIO() {
    m_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    m_feed_motor = new CANSparkMax(Ports.INTAKE_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */

  public void setSolenoid(DoubleSolenoid.Value state){
    m_solenoid.set(state);
  }

  public void setWheels(double volts){
    m_feed_motor.set(volts);
  }

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
