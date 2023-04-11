package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SwerveModule {
	private CANSparkMax driveMotor;
	private CANSparkMax steerMotor;


	private PIDController steerController;

	private CANCoder encoder;
	private final int module;

	private SwerveModulePosition modulePosition;

	private double speed;
	private double angle;

	public SwerveModule(
			int driveID,
			int steerID,
			int canCoderID,
			int canCoderOffset,
			int module // 1 for front left 2 for front right and so 3 for back left and 4 for back right
	)
	{
		driveMotor = new CANSparkMax(driveID, CANSparkMaxLowLevel.MotorType.kBrushless);
		steerMotor = new CANSparkMax(steerID, CANSparkMaxLowLevel.MotorType.kBrushless);

		encoder = new CANCoder(canCoderID);
		this.module = module;

		encoder.configMagnetOffset(canCoderOffset);

		driveMotor.restoreFactoryDefaults();
		steerMotor.restoreFactoryDefaults();

		driveMotor.setSmartCurrentLimit(40);
		steerMotor.setSmartCurrentLimit(40);

		driveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
		steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

		driveMotor.setInverted(false);
		steerMotor.setInverted(false);

		driveMotor.set(0);
		steerMotor.set(0);

		steerController = new PIDController(0,0,0); // FIXME tune

		modulePosition = new SwerveModulePosition(0,new Rotation2d(0));

		shuffleBoardInit();
	}

	public void setModule(double speed, double angle) // speed in motor percent and angle in degrees
	{
		this.speed = speed;
		this.angle = angle;

		steerMotor.set(
				steerController.calculate(encoder.getPosition(), angle)
		);

		driveMotor.set(speed);

		modulePosition = new SwerveModulePosition(driveMotor.get(), new Rotation2d(Math.toRadians(angle)));
	}

	public SwerveModulePosition getModulePosition() {
		return modulePosition;
	}

	private void shuffleBoardInit() {
		String moduleName;
		switch (module) {
			case 1:
				moduleName = "Front Left";
				break;
			case 2:
				moduleName = "Front Right";
				break;
			case 3:
				moduleName = "Back left";
				break;
			case 4:
				moduleName = "Back Right";
				break;
			default:
				moduleName = "How";
				break;
		}
		ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

		ShuffleboardLayout layout = tab.getLayout(moduleName, "List");

		layout.addDouble("Raw Encoder Value", () -> encoder.getAbsolutePosition());

		layout.addDouble("Speed", () -> driveMotor.getEncoder().getVelocity());

		layout.addDouble("Speed Set", () -> getModulePosition().distanceMeters);
		layout.addDouble("Angle", () -> getModulePosition().angle.getDegrees());


	}
}
