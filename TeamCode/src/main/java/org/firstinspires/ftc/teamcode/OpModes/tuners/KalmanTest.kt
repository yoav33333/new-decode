package org.firstinspires.ftc.teamcode.OpModes.tuners

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.extensions.pedro.PedroComponent
import org.firstinspires.ftc.teamcode.Pedro.Constants
import org.firstinspires.ftc.teamcode.Pedro.Constants.createFollower
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveCommands
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveHardware
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLight
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware
@TeleOp
class KalmanTest: TunerOpMode(LimeLight, PedroComponent(Constants::createFollower), DriveHardware) {
    override fun onStartButtonPressed() {
        TurretHardware.setAngle(0.0)
        DriveCommands.driverControlled.schedule()

    }
}