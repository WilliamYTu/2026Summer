package org.firstinspires.ftc.teamcode
import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Limelight

@Configurable
class Robot( val hardwareMap: HardwareMap, val telemetry: Telemetry) {
    val limelight = Limelight(hardwareMap, telemetry)
}