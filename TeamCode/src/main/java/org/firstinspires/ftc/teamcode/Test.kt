package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "Ball Detector")
class Test : LinearOpMode() {

    override fun runOpMode() {
        val limelight = Limelight(hardwareMap, telemetry)
        val robot = Robot(hardwareMap, telemetry)
        limelight.initLimelight()
        limelight.start()
        waitForStart()

        while (opModeIsActive()) {
            limelight.detectBalls()
            if (gamepad1.x) {
                robot.limelightToggle()
            }

            updateTelemetry(telemetry)
            telemetry.addData("Servo position", limelight.getServoPosition())
        }
    }
}