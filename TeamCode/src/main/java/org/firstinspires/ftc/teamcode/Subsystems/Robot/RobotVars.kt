package org.firstinspires.ftc.teamcode.Subsystems.Robot

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import org.firstinspires.ftc.teamcode.Util.SpindexerSlotState

@Configurable
object RobotVars {
    var goalPosRed = Vector(Pose(144.0,144.0))
    var goalPosBlue = Vector(Pose(0.0,144.0))
    var auto = false
    var goalPos = Vector(0.0, 0.0)
    var vectorFromTarget = Vector(0.0, 0.0)
    @JvmField var allianceColor = AllianceColor.RED
    @JvmField var randomization = Randomization.PPG
}

enum class AllianceColor {
    RED,
    BLUE
}
enum class Randomization(val value: Array<SpindexerSlotState>){
    PPG(arrayOf(SpindexerSlotState.PURPLE, SpindexerSlotState.PURPLE, SpindexerSlotState.GREEN)),
    PGP(arrayOf(SpindexerSlotState.PURPLE, SpindexerSlotState.GREEN, SpindexerSlotState.PURPLE)),
    GPP(arrayOf(SpindexerSlotState.GREEN, SpindexerSlotState.PURPLE, SpindexerSlotState.PURPLE))
}