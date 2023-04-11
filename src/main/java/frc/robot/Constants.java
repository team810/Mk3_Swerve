package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
	public static final class Drivetrain
	{
		public static final class frontLeft
		{
			public final static int STEER_ID = 1;
			public final static int DRIVE_ID = 2;

			public final static int CAN_CODER_ID = 14;
			public final static int CAN_CODER_OFFSET = 0;
		}
		public static final class frontRight
		{
			public final static int STEER_ID = 3;
			public final static int DRIVE_ID = 4;

			public final static int CAN_CODER_ID = 15;
			public final static int CAN_CODER_OFFSET = 0;
		}
		public static final class backLeft
		{
			public final static int STEER_ID = 5;
			public final static int DRIVE_ID = 6;

			public final static int CAN_CODER_ID = 16;
			public final static int CAN_CODER_OFFSET = 0;
		}
		public static final class backRight
		{
			public final static int STEER_ID = 7;
			public final static int DRIVE_ID = 8;

			public final static int CAN_CODER_ID = 17;
			public final static int CAN_CODER_OFFSET = 0;
		}

		public static final int MANUEL_DRIVE_MODE = 1;
		public static final int AUTO_DRIVE_MODE = 2;
		public static final int AUTO_ALINE = 3;
		public static final int AUTO_TURN = 4; // AUTO turn will still allow x and y movement

		public static int DRIVE_MODE = 0;

		public static final double MAX_SPEED = 4.8;

		public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.635;
		public static final double DRIVETRAIN_WHEELBASE_METERS = 0.635;
		public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
				// Front left
				new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
						DRIVETRAIN_WHEELBASE_METERS / 2.0),
				// Front right
				new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
						-DRIVETRAIN_WHEELBASE_METERS / 2.0),
				// Back left
				new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
						DRIVETRAIN_WHEELBASE_METERS / 2.0),
				// Back right
				new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
						-DRIVETRAIN_WHEELBASE_METERS / 2.0));
	}
}
