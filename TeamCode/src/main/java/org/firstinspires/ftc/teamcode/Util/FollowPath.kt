package org.firstinspires.ftc.teamcode.Util

import com.pedropathing.paths.PathChain
import dev.nextftc.core.commands.Command
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower

class FollowPath(
    var path: PathChain,
    var holdEnd:Boolean = true,
    var maxPower: Double = 1.0) : Command() {
    override val isDone: Boolean
        get() =!follower.isBusy || follower.isRobotStuck

    override fun start() {
        follower.followPath(path, maxPower, holdEnd)
    }
}