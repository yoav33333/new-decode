package org.firstinspires.ftc.teamcode.Subsystems.Robot

import com.qualcomm.hardware.lynx.LynxModule
import dev.nextftc.core.components.Component
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.allianceColor
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.goalPosBlue
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.goalPosRed
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.nominalVoltage
import kotlin.collections.forEach

object Robot: Component {
    val controlHub = lazy{ ActiveOpMode.hardwareMap.getAll(LynxModule::class.java)[0] }
    fun getVoltage():Double{
        return controlHub.value.getInputVoltage(VoltageUnit.VOLTS)
    }
    fun normalizePower(power: Double): Double{
        return power*nominalVoltage/getVoltage()
    }
    override fun preInit() {
        RobotVars.goalPos =( if (allianceColor == AllianceColor.RED) goalPosRed else goalPosBlue)
//        RobotVars.goalPos =( goalPosBlue)
    }

    override fun preUpdate() {
        MyTelemetry.addData("Goal Pos", RobotVars.goalPos)
        MyTelemetry.addData("Alliance Color", allianceColor)
//        MyTelemetry.addData("distance from goal", RobotVars.vectorFromTarget.magnitude)
    }


}