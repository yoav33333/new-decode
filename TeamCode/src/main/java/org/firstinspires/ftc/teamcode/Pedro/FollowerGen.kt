package org.firstinspires.ftc.teamcode.Pedro

import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.Pedro.Constants.generateFollower

object FollowerGen {
    @JvmStatic val followerGen = lazy{generateFollower(ActiveOpMode.hardwareMap)}
}