// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private CANSparkMax leftPivotMotor;
  private CANSparkMax rightPivotMotor;
  private CANSparkMax rollerMotor;
  private AbsoluteEncoder pivotEncoder;
  private PIDController pivotController;

  public Intake() {
    leftPivotMotor = new CANSparkMax(0, MotorType.kBrushless);
    rightPivotMotor = new CANSparkMax(0, MotorType.kBrushless);
    rollerMotor = new CANSparkMax(0, MotorType.kBrushless);
    pivotEncoder = leftPivotMotor.getAbsoluteEncoder(Type.kDutyCycle);

    leftPivotMotor.setIdleMode(IdleMode.kBrake);
    rightPivotMotor.setIdleMode(IdleMode.kBrake);

    rollerMotor.setIdleMode(IdleMode.kCoast);

    rightPivotMotor.follow(leftPivotMotor, true);

    leftPivotMotor.burnFlash();
    rightPivotMotor.burnFlash();

    pivotController = new PIDController(0, 0, 0);

    pivotEncoder.setPositionConversionFactor(IntakeConstants.kPivotGearRatio * 360);
    pivotEncoder.setZeroOffset(0); // TBD
  }

  public double getPivotAngle() {
    return pivotEncoder.getPosition() / IntakeConstants.kPivotGearRatio;
  }

  public void setPivotAngle(double targetDegrees) {
    pivotController.setSetpoint(targetDegrees);
  }

  public void intake() {
    rollerMotor.setVoltage(2);
  }

  public void extake() {
    rollerMotor.setVoltage(-2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leftPivotMotor.setVoltage(pivotController.calculate(getPivotAngle()));

    SmartDashboard.putNumber("Intake Pivot Angle", getPivotAngle());
  }
}