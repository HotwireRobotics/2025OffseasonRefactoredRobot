package frc.robot.subsystems;


import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;


public class DriveTrain extends SwerveDriveTrain {
	public DriveTrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
	}

	@Override
	public void periodic() {
		if (currentDriveState == DriveState.DRIVING){
			driveCommand().schedule();
		}
	}

	private DriveState currentDriveState = DriveState.STOPPED;

	public DriveState getCurrentState(){
		return currentDriveState;
	}

	public enum DriveState {
		STOPPED,
		DRIVING,
		AUTO,
		PATHFINDING,

		IDLE

	}

	public void disableDriverControls(){
		currentDriveState = DriveState.STOPPED;
		this.idle().schedule();
	}

	public void reenableDriverControls(){
		currentDriveState = DriveState.DRIVING;
	}


	public Command getPathToPoseCmd(Pose2d targetPose) {
		return new InstantCommand(()->currentDriveState = DriveState.PATHFINDING)
		.andThen(AutoBuilder.pathfindToPose(targetPose, Constants.constraints))
		.andThen(new InstantCommand(()->currentDriveState= DriveState.DRIVING));
	}

	public Command driveCommand() {
		return applyRequest(() -> Constants.drive.withVelocityX(-(Constants.driver.getLeftY()) * (Constants.driver.x().getAsBoolean() ? Constants.MaxSpeed / 2 : Constants.MaxSpeed)) 
		// Drive forward with
		// negative Y
		// (forward)
		.withVelocityY(-(Constants.driver.getLeftX()) * (Constants.driver.x().getAsBoolean() ? Constants.MaxSpeed / 2 : Constants.MaxSpeed)) // Drive left with negative X (left)
		.withRotationalRate(-Constants.driver.getRightX() * Constants.MaxAngularRate)); // Drive counterclockwise with negative X (left)
	}

}