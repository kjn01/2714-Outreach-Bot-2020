// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class Conveyor extends SubsystemBase {
  /** Creates a new Conveoyr. */
  private CANSparkMax horizontalConveyor;
  private CANSparkMax verticalConveyor;

  public Conveyor() {
    horizontalConveyor = new CANSparkMax(ConveyorConstants.kHorizontalConveyorMotorCanId, MotorType.kBrushless);
    verticalConveyor = new CANSparkMax(ConveyorConstants.kVerticalConveyorMotorCanId, MotorType.kBrushless);

    horizontalConveyor.setSmartCurrentLimit(30);
    verticalConveyor.setSmartCurrentLimit(30);

    horizontalConveyor.setIdleMode(IdleMode.kBrake);
    verticalConveyor.setIdleMode(IdleMode.kBrake);

    horizontalConveyor.setInverted(true);

  }

  public void intake() {
    horizontalConveyor.set(0.5);
    verticalConveyor.set(0.5);
  }

  public void extake() {
    horizontalConveyor.set(-0.5);
    verticalConveyor.set(-0.5);
  }

  public void stop() {
    horizontalConveyor.set(0);
    verticalConveyor.set(0);
  }

  public void horiz() {
    horizontalConveyor.set(0.5);
  }

  public void vert() {
    verticalConveyor.set(0.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
