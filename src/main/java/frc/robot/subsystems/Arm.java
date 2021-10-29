// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Arm extends SubsystemBase {
  private final double GrabberPower = 0.3;
  // Talons for grabbers
  private WPI_TalonSRX leftTalon;
  private WPI_TalonSRX rightTalon;
  private WPI_TalonSRX armTalon;

  private PIDController armTalonPID;
  private double kP = 0.0;
  private double kI = 0.0;
  private double kD = 0.0;

  private ArmFeedforward armFeedForward;
  private double kS = 0.0;
  private double kCos = 0.0;
  private double kV = 0.0;
  private double kA = 0.0;

  private double kTolerance = 0.0;
  private double kDerivativeTolerance = 0.0;

  private double angleSetPoint = 512; // From zero to 1024 ticks?
  private double velocitySetPoint = 1;

  public Arm() {
    leftTalon = new WPI_TalonSRX(Constants.ArmTalons.LeftTalonPort);
    leftTalon.configFactoryDefault();
    leftTalon.setInverted(false);
    rightTalon = new WPI_TalonSRX(Constants.ArmTalons.RightTalonPort);
    rightTalon.configFactoryDefault();
    rightTalon.setInverted(false);
    armTalon = new WPI_TalonSRX(Constants.ArmTalons.ArmTalonPort);
    armTalon.configFactoryDefault();
    armTalon.setInverted(false);
    armTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
    armTalonPID = new PIDController(kP, kI, kD);
    armTalonPID.setTolerance(kTolerance, kDerivativeTolerance);
    armFeedForward = new ArmFeedforward(kS, kCos, kV, kA);
  }
  /*
  public void setArmAngle(double angle, double velocity) {
    // PID does tuning while feedforward is a rough estimate added on to that
    armTalon.setVoltage(armFeedForward.calculate(angle, velocity) +
      armTalonPID.calculate(armTalon.getSelectedSensorPosition(0), angle));
  }
  */
  public void setArmPower(double power) {
    armTalon.set(ControlMode.PercentOutput, power);
  }

  public double getArmAngleInDegrees() {
    return (armTalon.getSelectedSensorPosition()/4096.0)*360.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // The feedforward is returned as a voltage as alluded
    // to in the setVoltage documentation
    //setArmAngle(angleSetPoint, velocitySetPoint);
    if (RobotContainer.getJoystick().getRawButton(1)) {
      leftTalon.set(ControlMode.PercentOutput, GrabberPower);
      rightTalon.set(ControlMode.PercentOutput, GrabberPower);
    } else {
      leftTalon.set(ControlMode.PercentOutput, 0.0);
      rightTalon.set(ControlMode.PercentOutput, 0.0);
    }
  }
}
