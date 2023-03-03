// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.SetArmExtension;
import frc.robot.commands.arm.ZeroArmPosition;
import frc.robot.commands.claw.ClawClose;
import frc.robot.commands.claw.ClawOpen;
import frc.robot.commands.claw.ClawSensorGrab;
import frc.robot.commands.drivetrain.DriveAtVelocity;
import frc.robot.commands.shoulder.SetShoulderPosition;
import frc.robot.commands.shoulder.ZeroShoulderPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickupPieceFromGround extends SequentialCommandGroup {
  /** Creates a new PickupPieceFromGround. */
  public PickupPieceFromGround() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ClawClose(),
      Commands.parallel(
        new ZeroShoulderPosition(),
        new ZeroArmPosition().andThen(new SetArmExtension(0.0))
      ),
      new SetShoulderPosition(-60.0),
      Commands.parallel(
        new SetArmExtension(0.34),
        new ClawSensorGrab(),
        new DriveAtVelocity(1.0)
      ),
      new DriveAtVelocity(0.0),
      // new ClawOpen(),
      // new WaitCommand(0.2),
      // new ClawSensorGrab(),
      Commands.parallel(
        new SetArmExtension(0.0),
        new SetShoulderPosition(-50.0)
      )
    );
  }
}
