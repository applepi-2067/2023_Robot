// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.arm.SetArmExtension;
import frc.robot.commands.shoulder.BlockUntilShoulderGreaterThan;
import frc.robot.commands.shoulder.SetShoulderPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreHighAuto extends ParallelCommandGroup {
  /** Creates a new ScoreHighAuto. */
  public ScoreHighAuto() {
    addCommands(
      new SetShoulderPosition(20.0),  // Forward is 20 deg  -- High scoring position
      new BlockUntilShoulderGreaterThan(0.0).andThen(
      new SetArmExtension(0.84)) // High scoring position
    );
  }
}
