package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.Supplier;

public class DriveToPose extends CommandBase {

    private DriveSubsystem drive;
    private Supplier<Pose2d> currentPose = drive::getCurrentPose;
    private final Pose2d targetPose;

    private ProfiledPIDController translationPID = new ProfiledPIDController(
            0.001, 0, 0,
            new TrapezoidProfile.Constraints(3 * 0.02, 3 * 0.02)
    );
    private ProfiledPIDController rotationPID = new ProfiledPIDController(
            0.001, 0, 0,
            new TrapezoidProfile.Constraints(3 * 0.02, 3 * 0.02)
    );


    public DriveToPose(DriveSubsystem drive, Pose2d targetPose) {
        this.drive = drive;
        this.targetPose = targetPose;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        translationPID.setTolerance(0.5);
        rotationPID.setTolerance(3);
    }

    @Override
    public void execute() {
        double distanceAwayX = currentPose.get().getX() - targetPose.getX();
        double distanceAwayY = currentPose.get().getY() - targetPose.getY();
        double distanceAway = Math.sqrt(Math.pow(distanceAwayX, 2) + Math.pow(distanceAwayY, 2));
        double angleOfDistance = Math.atan2(distanceAwayY, distanceAwayX);

        double translationOutput = translationPID.calculate(distanceAway, 0);


        double newRotation = rotationPID.calculate(currentPose.get().getRotation().getDegrees(), targetPose.getRotation().getDegrees());
        double newForward = translationOutput * Math.cos(angleOfDistance);
        double newStrafe = translationOutput * Math.sin(angleOfDistance);

        drive.fieldCentricDrive(newForward, newStrafe, newRotation);
    }

    @Override
    public void end(boolean interrupted) {
        drive.fieldCentricDrive(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return translationPID.atGoal() && rotationPID.atGoal();
    }
}
