package org.firstinspires.ftc.teamcode.Subsystems.Robot

import com.qualcomm.hardware.lynx.LynxModule
import dev.nextftc.core.components.Component
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.photoncore.Neutrino.Rev2MSensor.photoncore.PhotonCore

object Photon: Component {
    override fun preInit() {
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL)
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL)
        PhotonCore.experimental.setMaximumParallelCommands(8)
        PhotonCore.enable()
    }
    override fun postWaitForStart() = clearCache()

    override fun postUpdate() = clearCache()

    private fun clearCache() {
        PhotonCore.CONTROL_HUB.clearBulkCache()
        PhotonCore.EXPANSION_HUB.clearBulkCache()
    }
}