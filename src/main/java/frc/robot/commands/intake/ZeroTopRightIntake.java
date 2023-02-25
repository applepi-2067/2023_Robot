package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeExtensionMotor;
import frc.robot.utils.Util;

public class ZeroTopRightIntake extends CommandBase {
    private IntakeExtensionMotor intakeExtensionMotor;
    private final double SPEED = -0.6;
    private final double zeroMotorCurrent = 10.0; //AMPS

    private double averaged_current = 0.0;
    private final double AVG_GAIN = 0.8;
  
    public ZeroTopRightIntake() {
     intakeExtensionMotor = IntakeExtensionMotor.getInstance();
     
    }
  
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      intakeExtensionMotor.setSpeedRight(SPEED);
      if (intakeExtensionMotor.getRightMotorCurrent() >= zeroMotorCurrent){
        intakeExtensionMotor.setSpeedRight(0.0);
        intakeExtensionMotor.zeroRightEncoder();
      }
      
      averaged_current = Util.runningAverage(intakeExtensionMotor.getLeftMotorCurrent(), averaged_current,AVG_GAIN );
    }
  
    @Override
    public void end(boolean interrupted) {
        intakeExtensionMotor.zeroRightEncoder();
    }
    

    @Override
    public boolean isFinished() {
      return (averaged_current >= zeroMotorCurrent);
    }
  }
