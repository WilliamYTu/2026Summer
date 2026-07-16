package org.firstinspires.ftc.teamcode

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
            limelight.getBallNumber()
            if (gamepad1.dpad_down){
                robot.changeLimelightPos(-100.0)
            }
            if (gamepad1.dpad_up){
                robot.changeLimelightPos(100.0)
            }
            if (gamepad1.x){
                robot.limelightToggle()
            }

            telemetry.addData("Servo position", limelight.getServoPosition())
            limelight.returnTelemetry()
            telemetry.update()
        }
    }
}