// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private CANSparkMax leftShooterMotor;
  private CANSparkMax rightShooterMotor;

  private RelativeEncoder shooterEncoder;

  private PIDController shooterController;
  private SimpleMotorFeedforward shooterFeedforward;

  public Shooter() {
    leftShooterMotor = new CANSparkMax(ShooterConstants.kLeftShooterMotorCanId, MotorType.kBrushless);
    rightShooterMotor = new CANSparkMax(ShooterConstants.kRightShooterMotorCanId, MotorType.kBrushless);

    shooterEncoder = leftShooterMotor.getEncoder();

    rightShooterMotor.follow(leftShooterMotor, true);

    leftShooterMotor.setSmartCurrentLimit(60);
    rightShooterMotor.setSmartCurrentLimit(60);

    leftShooterMotor.setIdleMode(IdleMode.kCoast);
    rightShooterMotor.setIdleMode(IdleMode.kCoast);

    shooterController.setP(0.0006);
    shooterFeedforward = new SimpleMotorFeedforward(0, 0.13, 2.22);
  }

  public void setToShoot() {
    shooterController.setSetpoint(10);
  }

  public void setToStop() {
    shooterController.setSetpoint(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leftShooterMotor.setVoltage(shooterController.calculate(shooterEncoder.getVelocity()) + shooterFeedforward.calculate(10, 20));
  }
}
