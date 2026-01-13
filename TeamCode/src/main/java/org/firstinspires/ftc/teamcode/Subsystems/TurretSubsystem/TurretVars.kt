package org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem

import com.bylazar.configurables.annotations.Configurable

@Configurable
object TurretVars {
    @JvmField var servoRange = 270.0
    @JvmField var offset = .0
//270/360*255=255.75
}