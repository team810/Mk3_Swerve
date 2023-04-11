package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.HashMap;

public class Autos {

	SwerveAutoBuilder autoBuilder;
	private final DrivetrainSubsystem m_drive;
	private final HashMap<String, Command> eventMap = new HashMap<>();
	public Autos(DrivetrainSubsystem m_drive)
	{

		autoBuilder = new SwerveAutoBuilder(
				m_drive::getPose,
				m_drive::resetPos,
				Constants.Drivetrain.KINEMATICS,
				new PIDConstants(15,0,0),
				new PIDConstants(15,0,0),
				m_drive::setAutoStates,
				eventMap,
				false
		);
		this.m_drive = m_drive;
	}

	public Command genCommand(String path)
	{
		Command autoBuilderCommand = autoBuilder.fullAuto(PathPlanner.loadPath(path, new PathConstraints(.5,3)));

		return new SequentialCommandGroup(
				new InstantCommand(() -> Constants.Drivetrain.DRIVE_MODE = Constants.Drivetrain.AUTO_DRIVE_MODE),
				autoBuilderCommand,
				new InstantCommand(() -> Constants.Drivetrain.DRIVE_MODE = Constants.Drivetrain.AUTO_DRIVE_MODE)
		);
	}
}
