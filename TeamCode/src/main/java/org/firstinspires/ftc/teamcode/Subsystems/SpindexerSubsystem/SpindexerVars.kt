package org.firstinspires.ftc.teamcode.Subsystems.SpindexerSubsystem

import com.bylazar.configurables.annotations.Configurable
//import org.firstinspires.ftc.teamcode.Util.ColorRange
import org.firstinspires.ftc.teamcode.Util.HSVRange


@Configurable
object SpindexerVars {

    @JvmField var targetPosition = 0.0
    @JvmField var startIntakingStep = 5
    @JvmField var stuckTimeStart = 0.0
    @JvmField var wasStuck = false
    @JvmField var steps = 0
    @JvmField var maxRotation = 355.0
    @JvmField var degreesPerSlot = 66.5
    @JvmField var offset = 6.5
    @JvmField var offsetEnc = 0.0
    @JvmField var MulEnc = 1.1
    @JvmField var purpleRange = HSVRange(170.0,1000.0,0.0,1000.0,0.0,1000.0)
    @JvmField var greenRange = HSVRange(80.0,170.0,0.0,1000.0,0.0,1000.0)
    @JvmField var defaultColor = SpindexerSlotState.PURPLE
    @JvmField var state = State.RUN

}

enum class State{
    RESET,
    RUN,
    FIX
}
public enum class SpindexerSlotState(s: String) {
    EMPTY("E"),
    PURPLE("P"),
    GREEN("G"),
}