// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;

public abstract class Intake extends SubsystemBase {

  public DoubleSolenoid.Value solenoid_default, solenoid_current;

  public Commands commands;

  /** Creates a new ExampleSubsystem. */
  public Intake() {
    solenoid_default = DoubleSolenoid.Value.kOff;
    setSolenoidValue(null);

    this.commands = new Commands();
  }

  public void intake(){
    setWheels(IntakeConstants.INTAKE_VOLTS);
  }

  public void outtake(){
    setWheels(IntakeConstants.OUTTAKE_VOLTS);
  }

  public void stopFeed(){
    setWheels(0);
  }

  public void stop(){
    setSolenoidValue(null);
    stopFeed();
  }

  public void setSolenoidDefault(DoubleSolenoid.Value state){
    solenoid_default = state;
  }

  public void setSolenoidValue(DoubleSolenoid.Value state){
    solenoid_current = state;
    setSolenoid((state == null) ? solenoid_default : state);
  }

  abstract void setSolenoid(DoubleSolenoid.Value state);

  abstract void setWheels(double volts);

  public class Commands {

    public Command setDefault(){
      return new StartEndCommand(
        () -> setSolenoidDefault(DoubleSolenoid.Value.kForward), 
        () -> setSolenoidDefault(DoubleSolenoid.Value.kReverse));
    }
    
  }

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
