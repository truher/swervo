package frc.robot.commands;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.Drivetrain;

public class Goto extends SwerveControllerCommand {

    private Goto(Trajectory trajectory, Supplier<Pose2d> pose, SwerveDriveKinematics kinematics,
            PIDController xController, PIDController yController, ProfiledPIDController thetaController,
            Consumer<SwerveModuleState[]> outputModuleStates,
            Drivetrain requirements) {
        super(trajectory, pose, kinematics, xController, yController, thetaController,
                outputModuleStates,
                requirements);
    }

    public static Goto makeGoto(Drivetrain drivetrain) {
        TrajectoryConfig config = new TrajectoryConfig(2, 4).setKinematics(Drivetrain.kDriveKinematics);
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(1, 0)),
                new Pose2d(0, 0, new Rotation2d(Math.PI)),
                config);
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(20, 100);
        ProfiledPIDController thetaController = new ProfiledPIDController(1, 0, 0, constraints);
        return new Goto(
                exampleTrajectory,
                drivetrain::getPose,
                Drivetrain.kDriveKinematics,
                new PIDController(1, 0, 0),
                new PIDController(1, 0, 0),
                thetaController,
                drivetrain::setModuleStates,
                drivetrain);
    }

}
