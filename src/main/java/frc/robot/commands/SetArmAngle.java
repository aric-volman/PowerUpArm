// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ArmFeedforward;

public class SetArmAngle extends CommandBase {
  /** Creates a new SetArmAngle. */
  private double angle;

  private PIDController armTalonPID;
  private double kP = 0.0;
  private double kI = 0.0;
  private double kD = 0.0;

  private ArmFeedforward armFeedForward;
  private double kS = 0.0;
  private double kCos = 0.0;
  private double kV = 0.0;
  private double kA = 0.0;

  // private double kTolerance = 0.0;
  // private double kDerivativeTolerance = 0.0;

  private double velocitySetPoint = 0.0; // Keep this zero for an arm with an angle

  public SetArmAngle(double a) {
    // Use addRequirements() here to declare subsystem dependencies.
    angle = a;
    armTalonPID = new PIDController(kP, kI, kD);
    // armTalonPID.setTolerance(kTolerance, kDerivativeTolerance);
    armFeedForward = new ArmFeedforward(kS, kCos, kV, kA);
    addRequirements(RobotContainer.getArm());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armTalonPID.setSetpoint(angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.getArm().setArmPower(armFeedForward.calculate(angle, velocitySetPoint) +
    armTalonPID.calculate(RobotContainer.getArm().getArmAngleInDegrees()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
