package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.Supplier;


public class DeffultDriveCommand extends CommandBase {

	private final Supplier<XboxController> controller;
	private final DrivetrainSubsystem m_drive;
	private final ToTargetCommand m_toTargetCommand;
	
	public DeffultDriveCommand(Supplier<XboxController> controller, DrivetrainSubsystem m_drive) {
		this.controller = controller;
		this.m_drive = m_drive;

		m_toTargetCommand = new ToTargetCommand(m_drive);

		addRequirements(m_drive);
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		if (controller.get().getPOV() != -1)
		{
			Constants.Drivetrain.DRIVE_MODE = Constants.Drivetrain.AUTO_TURN;
			m_drive.setRotateTarget(controller.get().getPOV());
		} else if (controller.get().getPOV() == -1) {
			Constants.Drivetrain.DRIVE_MODE = Constants.Drivetrain.MANUEL_DRIVE_MODE;
		}
		if (RobotState.isAutonomous() && Constants.Drivetrain.DRIVE_MODE != Constants.Drivetrain.AUTO_ALINE) {
			Constants.Drivetrain.DRIVE_MODE = Constants.Drivetrain.AUTO_DRIVE_MODE;
		}

		m_drive.drive(controller.get().getLeftX(), controller.get().getLeftY(), controller.get().getRightX());
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {

	}
}
