package org.firstinspires.ftc.teamcode.OpModes.tuners

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.Gamepads
import org.firstinspires.ftc.teamcode.Pedro.Constants
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveCommands
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveCommands.resetIMU
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretCommands.moveToAngle
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretCommands.moveToGlobalAngle
import org.firstinspires.ftc.teamcode.Subsystems.Robot.Photon

@TeleOp(group = "tuning")
class TurretTuner :TunerOpMode(PedroComponent(Constants::createFollower), Photon,){
    init {
        Gamepads.gamepad2.a.whenBecomesTrue (moveToAngle(0.0))
        Gamepads.gamepad2.y.whenBecomesTrue (moveToAngle(270.0))
        Gamepads.gamepad2.b.whenTrue (moveToGlobalAngle(0.0))
        Gamepads.gamepad2.dpadDown.whenBecomesTrue (resetIMU )

    }
    override fun onStartButtonPressed() {
        DriveCommands.driverControlled.schedule()
//        lockOnGoal.schedule()
    }
}