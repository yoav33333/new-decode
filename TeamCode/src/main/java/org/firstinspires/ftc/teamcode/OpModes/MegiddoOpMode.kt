package org.firstinspires.ftc.teamcode.OpModes

import com.pedropathing.geometry.Pose
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeHardware
import org.firstinspires.ftc.teamcode.Pedro.Constants
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyTelemetry
import org.firstinspires.ftc.teamcode.Subsystems.Antony
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveHardware
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveVars.startingPose
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLight
import org.firstinspires.ftc.teamcode.Subsystems.Robot.AllianceColor
import org.firstinspires.ftc.teamcode.Subsystems.Robot.OpModeType
import org.firstinspires.ftc.teamcode.Subsystems.Robot.Photon
import org.firstinspires.ftc.teamcode.Subsystems.Robot.Robot
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterHardware
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferHardware
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware
import org.firstinspires.ftc.teamcode.Util.LoopTimer

open class MegiddoOpMode(allianceColor: AllianceColor, opModeType: OpModeType): NextFTCOpMode() {
    init {
        RobotVars.allianceColor = allianceColor
        RobotVars.opModeType = opModeType
        addComponents(
            BindingsComponent,
            BulkReadComponent,
            LoopTimer,
//            Photon,
            MyTelemetry,
            PedroComponent(Constants::createFollower),
            LimeLight,
            IntakeHardware,
            TransferHardware,
            SpindexerHardware,
            ShooterHardware,
            DriveHardware,
            TurretHardware,
            Robot,
            Antony
        )
    }

    override fun onStop() {
        startingPose = follower.pose
    }
}