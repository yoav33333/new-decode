package org.firstinspires.ftc.teamcode.Subsystems.Robot

import dev.nextftc.core.components.Component
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.allianceColor
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.goalPosBlue
import org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.goalPosRed

object Robot: Component {
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