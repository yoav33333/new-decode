package org.firstinspires.ftc.teamcode.OpModes.tuners

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.Gamepads
import org.firstinspires.ftc.teamcode.Pedro.Constants
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveCommands
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveCommands.resetIMU
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLight
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyTelemetry
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretCommands.moveToAngle
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretCommands.moveToAngler
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretCommands.moveToGlobalAngle
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware.getAngle
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware.getGlobalHeading
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware.getPosition
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware.getPositionServo
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretState
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.state
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretVars.targetPosition

//import org.firstinspires.ftc.teamcode.Subsystems.Robot.Photon

@TeleOp(group = "tuning")
class TurretTuner :TunerOpMode(PedroComponent(Constants::createFollower)){
    init {
        Gamepads.gamepad2.a.whenBecomesTrue (moveToAngler(0.0))
        Gamepads.gamepad2.y.whenBecomesTrue (moveToAngler(90.0))
        Gamepads.gamepad2.b.whenBecomesTrue (moveToAngler(-90.0))
//        Gamepads.gamepad2.b.whenTrue (moveToGlobalAngle(0.0))
        Gamepads.gamepad2.dpadDown.whenBecomesTrue (resetIMU )
//        Gamepads.gamepad2.rightBumper.whenTrue{}

    }
    override fun onStartButtonPressed() {
        DriveCommands.driverControlled.schedule()
//        lockOnGoal.schedule()
        state = TurretState.TrackingAprilTags
    }

    override fun onUpdate() {
        MyTelemetry.addData("Turret Servo Position", getPosition())
        MyTelemetry.addData("Turret Servo angle", getAngle())
        MyTelemetry.addData("Turret Servo realpos", getPositionServo())
        MyTelemetry.addData("Turret Servo angle2", ((getPosition()/8192)*360)/5)
//        MyTelemetry.addData("Turret Vol", turretEncoder.value.getVoltage())
        MyTelemetry.addData("Turret Target Position", targetPosition)
//        MyTelemetry.addData("Turret vol", getEncoderPosition())
        MyTelemetry.addData("Turret Global Heading", getGlobalHeading())
    }
}