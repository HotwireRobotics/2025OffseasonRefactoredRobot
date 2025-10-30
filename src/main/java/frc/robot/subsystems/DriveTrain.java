package frc.robot.subsystems;


import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;


public class DriveTrain extends SwerveDriveTrain {

	/**
	 * Constructor :)
	 * 
	 * @param drivetrainConstants
	 * @param modules
	 */
	public DriveTrain() {
		// Insert tuner constants.
        super(
			TunerConstants.DrivetrainConstants,
			TunerConstants.FrontLeft, TunerConstants.FrontRight,
			TunerConstants.BackLeft,  TunerConstants.BackRight
		);
	}

	/**
	 * Defines periodic behavior.
	 */
	@Override
	public void periodic() {
		if (currentDriveState == DriveState.DRIVING){
			driveCommand().schedule();
		}

		// Log module positions
		SwerveModuleState[] states = getState().ModuleStates;
		for (int i = 0; i < states.length; i++) {
			Rotation2d angle = states[i].angle;
			SmartDashboard.putString(

						"Module "+i+" Rotation",
				String.valueOf(angle.getDegrees())+'°'

			);
		}

		// Log robot pose
		Pose2d pose = getState().Pose;
		SmartDashboard.putNumber("Drivetrain.x", pose.getX());
		SmartDashboard.putNumber("Drivetrain.y", pose.getY());
		SmartDashboard.putNumber("Drivetrain.r", pose.getRotation().getDegrees());
	}

	//TODO Implement post-merge
	// /**
	//  * Supply current drive state to <code>Systerface</code>.
	//  */
	// @Override
	// public DriveState getCurrentState() {
	// 	return currentDriveState;
	// }

	/**
	 * Subsystem drive states.
	 */
	public enum DriveState {
		STOPPED,
		DRIVING,
		AUTO,
		PATHFINDING,

		IDLE

	}

	/**
	 * Instanciate current drive state.
	 */
	private DriveState currentDriveState = DriveState.STOPPED;

	/**
	 * Disable driver controls.
	 */
	public void disableDriveControls(){
		currentDriveState = DriveState.STOPPED;
		this.idle().schedule();
	}

	/**
	 * Enable driver controls.
	 */
	public void enableDriveControls(){
		currentDriveState = DriveState.DRIVING;
	}

	/**
	 * Command to pathfind to a target pose.
	 * 
	 * @param targetPose The target pose to pathfind to.
	 */
	public Command getPathCommand(Pose2d targetPose) {
		return new InstantCommand(() -> currentDriveState = DriveState.PATHFINDING)
			.andThen(AutoBuilder.pathfindToPose(targetPose, Constants.constraints))
			.andThen(new InstantCommand(() -> currentDriveState = DriveState.DRIVING));
	}

	public Command zeroHeadingCommand() {
		return runOnce(() -> seedFieldCentric());
	}

	/**
	 * Teleop drive command with joystick inputs.
	 */
	public Command driveCommand() {
		return applyRequest(() -> 
			Constants.drive
				.withVelocityX(-(Constants.driver.getLeftY()) * 
					(Constants.driver.x().getAsBoolean() ? Constants.MaxSpeed / 2 : Constants.MaxSpeed))
				.withVelocityY(-(Constants.driver.getLeftX()) * 
					(Constants.driver.x().getAsBoolean() ? Constants.MaxSpeed / 2 : Constants.MaxSpeed))
				.withRotationalRate(-Constants.driver.getRightX() * Constants.MaxAngularRate)
		);
	}	
}