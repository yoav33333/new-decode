package org.firstinspires.ftc.teamcode.OpModes

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveVars
import org.firstinspires.ftc.teamcode.Subsystems.Robot.AllianceColor
import org.firstinspires.ftc.teamcode.Subsystems.Robot.OpModeType
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.findPattern
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.scanCommand
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.shootingCommand
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.resetingSeq
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.isFull
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware.cachedVelocity
import org.firstinspires.ftc.teamcode.Util.FollowPath
import org.firstinspires.ftc.teamcode.Util.Util.Cycle
import org.firstinspires.ftc.teamcode.Util.Util.createPath
import kotlin.time.Duration.Companion.seconds

@Autonomous
//@Configurable
open class AutoBase(allianceColor: AllianceColor) : MegiddoOpMode(allianceColor, OpModeType.AUTO){
    open val startingPose = Pose()
    open val finishPose = Pose()
    open fun auto(): SequentialGroup = SequentialGroup()
    init {
    }
    val scan = SequentialGroup(
        resetingSeq,
        scanCommand.raceWith(WaitUntil{ isFull() })
    )
    open fun initializePaths(){}
    override fun onInit(){
        DriveVars.startingPose = startingPose
        initializePaths()
        scan.setRequirements(emptySet())
        scan.schedule()
    }
    override fun onStartButtonPressed() {

        val auto = auto()
        auto.setRequirements(this)
        auto.schedule()
        SequentialGroup(
            Delay(29.5.seconds),
            InstantCommand{
                LambdaCommand().setRequirements(this).schedule()
                shootingCommand.cancel()
                follower.holdPoint(finishPose)
            }
        ).schedule()
    }

    override fun onStop() {
        follower.breakFollowing()
        follower.startTeleopDrive()
    }

}