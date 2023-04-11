package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import lib.MoreMath;

public class DrivetrainSubsystem implements Subsystem {

	SwerveModule front_left;
	SwerveModule front_right;
	SwerveModule back_left;
	SwerveModule back_right;

	AHRS gyro;

	SwerveDriveKinematics kinematics;
	SwerveModulePosition[] modulePositions = {null, null, null, null};
	SwerveDrivePoseEstimator poseEstimator;
	PIDController rotateController = new PIDController(.5,0,0);

	private ChassisSpeeds autoAline;
	private SwerveModuleState[] autoStates;
	private double rotateTarget;

	public DrivetrainSubsystem() {
		gyro = new AHRS();

		front_left = new SwerveModule(
				Constants.Drivetrain.frontLeft.DRIVE_ID,
				Constants.Drivetrain.frontLeft.STEER_ID,
				Constants.Drivetrain.frontLeft.CAN_CODER_ID,
				Constants.Drivetrain.frontLeft.CAN_CODER_OFFSET,
				1
		);
		front_right = new SwerveModule(
				Constants.Drivetrain.frontRight.DRIVE_ID,
				Constants.Drivetrain.frontRight.STEER_ID,
				Constants.Drivetrain.frontRight.CAN_CODER_ID,
				Constants.Drivetrain.frontRight.CAN_CODER_OFFSET,
				2
		);
		 back_left = new SwerveModule(
				Constants.Drivetrain.backLeft.DRIVE_ID,
				Constants.Drivetrain.backLeft.STEER_ID,
				Constants.Drivetrain.backLeft.CAN_CODER_ID,
				Constants.Drivetrain.backLeft.CAN_CODER_OFFSET,
				3
		);
		back_right = new SwerveModule(
				Constants.Drivetrain.backRight.DRIVE_ID,
				Constants.Drivetrain.backRight.STEER_ID,
				Constants.Drivetrain.backRight.CAN_CODER_ID,
				Constants.Drivetrain.backRight.CAN_CODER_OFFSET,
				4
		);
		modulePositions[0] = front_left.getModulePosition();
		modulePositions[1] = front_right.getModulePosition();
		modulePositions[2] = back_left.getModulePosition();
		modulePositions[3] = back_right.getModulePosition();

		kinematics = Constants.Drivetrain.KINEMATICS;

		poseEstimator = new SwerveDrivePoseEstimator(kinematics, getGyroRotation(), modulePositions, new Pose2d());

		rotateController.enableContinuousInput(-180, 180);

	}

	public void drive(double x, double y, double rotate)
	{
		x = x * Constants.Drivetrain.MAX_SPEED;
		y = y * Constants.Drivetrain.MAX_SPEED;
		rotate = rotate * Constants.Drivetrain.MAX_SPEED;

		SwerveModuleState[] states;
//		Constants.Drivetrain.DRIVE_MODE = Constants.Drivetrain.MANUEL_DRIVE_MODE;
		if (Constants.Drivetrain.DRIVE_MODE == Constants.Drivetrain.MANUEL_DRIVE_MODE)
		{
			ChassisSpeeds speeds = new ChassisSpeeds(x, y, rotate);
			states = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(speeds,getGyroRotation()));
			
		} else if (Constants.Drivetrain.DRIVE_MODE == Constants.Drivetrain.AUTO_TURN) {
			double rotateSpeed;


			rotateSpeed = rotateController.calculate(getGyroRotation().getDegrees(), rotateTarget - 180);
			rotateSpeed = rotateSpeed * Constants.Drivetrain.MAX_SPEED;
			rotateSpeed = MoreMath.MinMax(rotateSpeed, 2,-2);

			ChassisSpeeds speeds = new ChassisSpeeds(x, y, rotateSpeed);

			states = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getGyroRotation()));
		} else if (Constants.Drivetrain.DRIVE_MODE == Constants.Drivetrain.AUTO_ALINE) {
			states = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(autoAline, getGyroRotation()));
		}else if(Constants.Drivetrain.DRIVE_MODE == Constants.Drivetrain.AUTO_DRIVE_MODE){
			states = autoStates;
		}else{
			states = kinematics.toSwerveModuleStates(new ChassisSpeeds(0,0,0));
		}


		setModuleStates(states);
	}

	private void setModuleStates(SwerveModuleState[] moduleStates)
	{
		front_left.setModule(
				moduleStates[0].speedMetersPerSecond,
				moduleStates[0].angle.getDegrees()
		);
		front_right.setModule(
				moduleStates[0].speedMetersPerSecond,
				moduleStates[0].angle.getDegrees()
		);
		back_left.setModule(
				moduleStates[0].speedMetersPerSecond,
				moduleStates[0].angle.getDegrees()
		);
		back_right.setModule(
				moduleStates[0].speedMetersPerSecond,
				moduleStates[0].angle.getDegrees()
		);
	}

	private void teleopPeriodic()
	{

	}

	private void autoPeriodic()
	{

	}

	@Override
	public void periodic() {
		if (RobotState.isTeleop())
		{
			teleopPeriodic();
		}
		if (RobotState.isAutonomous())
		{
			autoPeriodic();
		}
		if (RobotState.isEnabled())
		{
			modulePositions[0] = front_left.getModulePosition();
			modulePositions[1] = front_right.getModulePosition();
			modulePositions[2] = back_left.getModulePosition();
			modulePositions[3] = back_right.getModulePosition();

			poseEstimator.update(getGyroRotation(), modulePositions);
		}
	}

	public Rotation2d getGyroRotation()
	{
		return gyro.getRotation2d();
	}
	public void resetGyro()
	{
		gyro.zeroYaw();
	}
	public void setRotateTarget(double mRotateTarget) {
		rotateTarget = mRotateTarget;
	}
	public boolean atSetpoint(){return rotateController.atSetpoint();}

	public void setAutoAline(ChassisSpeeds mAutoAline) {
		autoAline = mAutoAline;
	}
	public Pose2d getPose()
	{
		return poseEstimator.getEstimatedPosition();
	}
	public void resetPos(Pose2d pos)
	{
		poseEstimator.resetPosition(getGyroRotation(),modulePositions, pos);
	}
	public void setAutoStates(SwerveModuleState[] mAutoStates) {
		autoStates = mAutoStates;
	}
}

