// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainVideoExample extends SubsystemBase {
  TalonSRX motorLeft1 = new TalonSRX(Constants.MOTOR_LEFT_1_ID);
  TalonSRX motorRight1 = new TalonSRX(Constants.MOTOR_RIGHT_1_ID);
  TalonSRX motorLeft2 = new TalonSRX(Constants.MOTOR_LEFT_2_ID);
  TalonSRX motorRight2 = new TalonSRX(Constants.MOTOR_RIGHT_2_ID);
  /** Creates a new DriveTrainVideoExample. */
  public DriveTrainVideoExample() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  //these methods make it so that the two left or two right motors cannot be at different speeds, causing issues
  public void setLeftMotors(double speed){
    motorLeft1.set(ControlMode.PercentOutput, -speed);
    motorLeft2.set(ControlMode.PercentOutput, -speed);
  }
  //one MUST be negative to ensure no errors

  public void setRightMotors(double speed){
    motorRight1.set(ControlMode.PercentOutput, speed);
    motorRight2.set(ControlMode.PercentOutput, speed);
  }


}
