// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.Constants;
import frc.robot.commands.arm.BlockUntilArmLessThan;
import frc.robot.commands.arm.SetArmExtension;
import frc.robot.commands.arm.ZeroArmPosition;
import frc.robot.commands.chargestation.DriveVelocityUntilDistance;
import frc.robot.commands.chargestation.BalanceOnCharge;
import frc.robot.commands.shoulder.InitShoulderZero;
import frc.robot.commands.shoulder.SetShoulderPosition;
import frc.robot.commands.shoulder.ZeroShoulderPosition;
import frc.robot.commands.waist.SetWaistPosition;
import frc.robot.commands.waist.ZeroWaistPosition;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.PresetPoses;
import frc.robot.commands.claw.ClawClose;
import frc.robot.commands.claw.ClawOpen;
import frc.robot.commands.claw.SetClawBeltSpeed;
import frc.robot.commands.drivetrain.BlockUntilDistanceTraveled;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CenterStartRoutine extends SequentialCommandGroup {
  public CenterStartRoutine(boolean isBlue) {
    Pose2d m_initalPose2d = PresetPoses.InitialPoses.Blue.CENTER_POSE2D;
    if (!isBlue) {
      m_initalPose2d = PresetPoses.InitialPoses.Red.CENTER_POSE2D;
    }

    Drivetrain.getInstance().setOdometryPose2d(m_initalPose2d);

    addCommands(
      // Grip cone.
      Commands.parallel(
        new ClawClose(),
        new SetClawBeltSpeed(() -> {return 0.2;})
      ),
      
      // Zero arm extension and shoulder angle
      Commands.parallel(
        new InitShoulderZero().andThen(new ZeroShoulderPosition()),
        new ZeroArmPosition().andThen(new SetArmExtension(0.0))
      ),

      // Score high auto.      
      new ScoreHighAuto(),
      new WaitCommand(0.1),

      Commands.parallel(
        // Retract arm and shoulder.
        new SetArmExtension(0.0),

        // Wait until arm is in safety zone, and then stow and drive back for mobility.
        new BlockUntilArmLessThan(Constants.Poses.ArmExtensions.SAFE_ROTATION).andThen(
          Commands.parallel(
            new SetShoulderPosition(Constants.Poses.ShoulderAngles.STOW),
            new DriveVelocityUntilDistance(-0.5, 4.0)
          )
        )
      ),

      // Drive forwards until we get on the charge station.
      new DriveVelocityUntilDistance(0.6, 1.45),
    
      Commands.parallel(
        new BalanceOnCharge(),
        new ZeroWaistPosition().andThen(new SetWaistPosition(0.0))
      )
    );
  }
}