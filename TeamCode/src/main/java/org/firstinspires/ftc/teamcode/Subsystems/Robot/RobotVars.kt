package org.firstinspires.ftc.teamcode.Subsystems.Robot

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import org.firstinspires.ftc.teamcode.Util.SpindexerSlotState
import org.firstinspires.ftc.teamcode.Util.Util.mmToInches
@Configurable
object RobotVars {
    var goalPosRed = Vector(Pose(142.75,142.75))
    var goalPosBlue = Vector(Pose(1.25,142.75))
    var obelisk = Vector(Pose(0.0,145.0))
    var resetPosBlue = Pose(133.5,10.4153543307, Math.toRadians(-90.0))
    var resetPosRed = resetPosBlue.mirror()
    var auto = false
    @JvmField var nominalVoltage = 12.0
    var goalPos = Vector(0.0, 0.0)
    var vectorFromTarget = Vector(0.0, 0.0)
    var deltaVec = Vector(0.0, 0.0)
    @JvmField var allianceColor = AllianceColor.RED
    @JvmField var randomization = Randomization.PPG
    @JvmField var randomizationOffset = 0
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

