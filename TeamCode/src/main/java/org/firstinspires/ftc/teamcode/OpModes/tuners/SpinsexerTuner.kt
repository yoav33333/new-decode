package org.firstinspires.ftc.teamcode.OpModes.tuners

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.ftc.Gamepads
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeCommands
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem.IntakeHardware
//import org.firstinspires.ftc.teamcode.Subsystems.Robot.Photon
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.intakeCommand
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.resetingSeq
//import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotCommands.shootingCommand
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.rotate
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.runIntakeCycle
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerCommands.transferAll
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.getColorInIntake
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.spindexerServo
import org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem.SpindexerHardware.tracker
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferCommands.runTransfer
import org.firstinspires.ftc.teamcode.Subsystems.TransferSubsystem.TransferCommands.stopTransfer
import org.firstinspires.ftc.teamcode.Util.SpindexerTracker
import org.firstinspires.ftc.teamcode.Util.UtilCommands.RepeatCommand
import kotlin.time.Duration.Companion.seconds

@TeleOp(group = "tuning")
class SpinsexerTuner: TunerOpMode(SpindexerHardware, IntakeHardware) {
    init {
        Gamepads.gamepad2.rightBumper
            .whenBecomesTrue(rotate(1))
        Gamepads.gamepad2.leftBumper
            .whenBecomesTrue(rotate(-1))
        Gamepads.gamepad2.a.whenBecomesTrue (intakeCommand )
        Gamepads.gamepad2.b.whenBecomesTrue (runIntakeCycle )
        Gamepads.gamepad2.leftStickButton.whenBecomesTrue{tracker = SpindexerTracker()}
        Gamepads.gamepad2.dpadDown.whenBecomesTrue(
            ParallelGroup(
                runTransfer,
                        transferAll(Delay(0.5.seconds)),

                    ).then(stopTransfer))
        RepeatCommand(InstantCommand { getColorInIntake() }).schedule()
                        Gamepads . gamepad2 . dpadUp . whenBecomesTrue ((IntakeCommands.intake))
                    .whenBecomesFalse(IntakeCommands.stopIntake)
        Gamepads.gamepad2.y.whenBecomesTrue (resetingSeq)

    }

    override fun onInit() {
//        resetingSeq.schedule()
//        spindexerServo.value.position = 0.5
    }
    override fun onUpdate() {
//        var hsv = colorSensor2.value.getHSV()
//        MyTelemetry.addData("color sensor 2", hsv.toString())
//        if (purpleRange.inRange(hsv)) {
//            MyTelemetry.addData("Intake Color Sensor 2", "Purple")
////            return SpindexerSlotState.PURPLE
//        } else if (greenRange.inRange(hsv)) {
//            MyTelemetry.addData("Intake Color Sensor 2", "Green")
////            return SpindexerSlotState.GREEN
//        } else {
//            MyTelemetry.addData("Intake Color Sensor 2", "Empty")
////            return SpindexerSlotState.EMPTY
//        }
//        hsv = colorSensor2.value.getHSV()
//        MyTelemetry.addData("color sensor 2", hsv.toString())
//        hsv = colorSensor1.value.getHSV()
//
//        MyTelemetry.addData("color sensor 1", hsv.toString())
    }
}