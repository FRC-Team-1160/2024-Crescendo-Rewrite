// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;

public abstract class Intake extends SubsystemBase {

  /**The default setting of the solenoid determined by the switch on the driver dashboard. */
  public DoubleSolenoid.Value solenoid_default;
  /**The current setting of the solenoid controlled by commands. This overrides solenoid_default*/
  public DoubleSolenoid.Value solenoid_current;

  public Commands commands;

  public Intake() {
    solenoid_default = DoubleSolenoid.Value.kOff;
    setSolenoidValue(null);

    this.commands = new Commands();
  }

  /**
   * Starts the intake wheels.
   */

  public void intake(){
    setWheels(IntakeConstants.INTAKE_VOLTS);
  }

  /**
   * Starts the intake wheels in reverse.
   */

  public void outtake(){
    setWheels(IntakeConstants.OUTTAKE_VOLTS);
  }

  /**
   * Stops the wheels.
   */

  public void stopFeed(){
    setWheels(0);
  }

  /**
   * Stops all intake activity. This resets the solenoid to the default position.
   */

  public void stop(){
    setSolenoidValue(null);
    stopFeed();
  }

  /**
   * Sets the solenoid's default state. This should primarily be done by the dashboard switch.
   * @param state The desired state.
   */

  public void setSolenoidDefault(DoubleSolenoid.Value state){
    solenoid_default = state;
  }

  /**
   * Sets the solenoid's current state. Passing null will allow the solenoid to return to the default state.
   * @param state The desired state.
   */

  public void setSolenoidValue(DoubleSolenoid.Value state){
    solenoid_current = state;
    setSolenoid((state == null) ? solenoid_default : state);
  }

  protected abstract void setSolenoid(DoubleSolenoid.Value state);

  protected abstract void setWheels(double volts);

  public class Commands {

    /**
     * Constructs a command that sets the solenoid default state to be triggered by the dashboard switch.
     * @return The dashboard switch Command object.
     */

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
