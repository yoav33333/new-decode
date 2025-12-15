package org.firstinspires.ftc.teamcode.OpModes.tuners

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.Pedro.Constants
import org.firstinspires.ftc.teamcode.Pedro.Constants.createFollower
import org.firstinspires.ftc.teamcode.Subsystems.Antony
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeCommands
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeHardware
import org.firstinspires.ftc.teamcode.Subsystems.LL.LimeLight
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyTelemetry
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.shootingCommand
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterHardware
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.rotate
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.runIntakeSeq
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.colorSensor1
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.colorSensor2
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.getColorInIntake
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.greenRange
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerVars.purpleRange
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferHardware
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem.TurretHardware
import org.firstinspires.ftc.teamcode.Util.LoopTimer
import org.firstinspires.ftc.teamcode.Util.SpindexerSlotState
import org.firstinspires.ftc.teamcode.Util.UtilCommands.LoopingCommand

@TeleOp(group = "tuning")
class SpinsexerTuner: TunerOpMode(SpindexerHardware) {
    init {
        Gamepads.gamepad2.rightBumper
            .whenBecomesTrue(rotate(1))
        Gamepads.gamepad2.leftBumper
            .whenBecomesTrue(rotate(-1))
        Gamepads.gamepad2.a.whenBecomesTrue { runIntakeSeq }
        LoopingCommand(InstantCommand{getColorInIntake()}).schedule()
        Gamepads.gamepad2.dpadUp.whenBecomesTrue { LoopingCommand(IntakeCommands.intake)}

    }

    override fun onUpdate() {
        var hsv = colorSensor2.value.getHSV()
        MyTelemetry.addData("color sensor 2", hsv.toString())
        if (purpleRange.inRange(hsv)) {
            MyTelemetry.addData("Intake Color Sensor 2", "Purple")
//            return SpindexerSlotState.PURPLE
        } else if (greenRange.inRange(hsv)) {
            MyTelemetry.addData("Intake Color Sensor 2", "Green")
//            return SpindexerSlotState.GREEN
        } else {
            MyTelemetry.addData("Intake Color Sensor 2", "Empty")
//            return SpindexerSlotState.EMPTY
        }
        hsv = colorSensor2.value.getHSV()
        MyTelemetry.addData("color sensor 2", hsv.toString())
        hsv = colorSensor1.value.getHSV()

        MyTelemetry.addData("color sensor 1", hsv.toString())
    }
}