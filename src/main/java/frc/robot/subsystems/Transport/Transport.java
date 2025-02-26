// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Transport;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.TransportConstants;

public abstract class Transport extends SubsystemBase {

  public boolean note_stored;

  public Timer color_refresh;

  public double prox;

  public Transport(){

  }

  /**
   * Turns the transport mechanism on.
   */

  public void transportOn(){
    setWheels(TransportConstants.WHEEL_SPEED_VOLTS);
    setBelt(TransportConstants.BELT_SPEED_VOLTS);
  }

  /**
   * Turns the transport mechanism off.
   */

  public void transportOff(){
    setWheels(0);
    setBelt(0);
  }

  /**
   * Turns the transport mechanism on in reverse.
   */

  public void outtake(){
    setWheels(-TransportConstants.WHEEL_SPEED_VOLTS);
    setBelt(-TransportConstants.BELT_SPEED_VOLTS);
  }

  abstract void setWheels(double volts);

  abstract void setBelt(double volts);

  abstract double readColorProx();

  abstract boolean getSwitchPressed();

  @Override
  public void periodic() {
    if (color_refresh.hasElapsed(0.1) && prox != 0) {
      color_refresh.restart();
      readColorProx();
    }
    note_stored = (prox > TransportConstants.PROX_THRESHOLD) || getSwitchPressed();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
