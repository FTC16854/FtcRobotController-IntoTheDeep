package org.firstinspires.ftc.teamcode.commandBased.commands._auto;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commandBased.Constants;
import org.firstinspires.ftc.teamcode.commandBased.commands._groups.LiftMoveRotateArm;
import org.firstinspires.ftc.teamcode.commandBased.commands._groups.ScoreCone;
import org.firstinspires.ftc.teamcode.commandBased.commands._rr.FollowTrajectorySequenceAsync;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.AutoSubsystems;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

public class InitialMoveMed extends SequentialCommandGroup {

    public InitialMoveMed(
            AutoSubsystems subsystems,
            TrajectorySequence traj
    ) {
        addCommands(
                new ParallelCommandGroup(
                        new LiftMoveRotateArm(
                                subsystems.getEle(),
                                subsystems.getArm(),
                                subsystems.getRot(),
                                Constants.ARM_ANGLE_BACK,
                                Constants.ELE_HIGH,
                                Constants.ROTATOR_BACK
                        ),
                        new FollowTrajectorySequenceAsync(subsystems.getDrive(), traj)
                ),
                new ScoreCone(subsystems.getArm(), subsystems.getRot(), subsystems.getIntake())
        );
    }
}
