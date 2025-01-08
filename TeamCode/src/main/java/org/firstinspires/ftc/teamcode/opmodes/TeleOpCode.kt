package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.IMU
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand
import com.rowanmcalpin.nextftc.core.control.controllers.AngularController
import com.rowanmcalpin.nextftc.core.control.controllers.Controller
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController
import com.rowanmcalpin.nextftc.ftc.OpModeData
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx
import com.rowanmcalpin.nextftc.pedro.PedroOpMode
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.MecanumDriverControlledFixed
import org.firstinspires.ftc.teamcode.StatefulSupplier
import org.firstinspires.ftc.teamcode.subsystems.ClawPivot
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlide
import org.firstinspires.ftc.teamcode.subsystems.Lift


@TeleOp(name = "Into The Deep TeleOp")
class TeleOpCode: PedroOpMode(Lift, HorizontalSlide, ClawPivot) {

    enum class HeadingState {
        GAMEPAD,
        PID
    }

    private lateinit var leftFront: MotorEx
    private lateinit var leftRear: MotorEx
    private lateinit var rightFront: MotorEx
    private lateinit var rightRear: MotorEx

    lateinit var motors: Array<MotorEx>

    lateinit var imu: IMU

    lateinit var driverControlled: MecanumDriverControlledFixed

    lateinit var headingSupplier: StatefulSupplier<HeadingState, Float>

    val headingPID: Controller = AngularController(PIDFController(1.0))

    override fun onInit() {
        headingPID.target = Math.PI

        leftFront = MotorEx("frontLeftMotor")
        leftRear = MotorEx("backLeftMotor")
        rightRear = MotorEx("backRightMotor")
        rightFront = MotorEx("frontRightMotor")

        motors = arrayOf(leftFront, rightFront, leftRear, rightRear)

        leftFront.direction = DcMotorSimple.Direction.FORWARD
        leftRear.direction = DcMotorSimple.Direction.REVERSE
        rightFront.direction = DcMotorSimple.Direction.FORWARD
        rightRear.direction = DcMotorSimple.Direction.FORWARD

        imu = hardwareMap.get(IMU::class.java, "imu")
        imu.initialize(IMU.Parameters(
            RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
            )))
        imu.resetYaw()

        headingSupplier = StatefulSupplier(
            mapOf(
                HeadingState.GAMEPAD to { gamepadManager.gamepad1.rightStick.x },
                HeadingState.PID to {
                    headingPID.calculate(
                        imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
                    ).toFloat()
                }),
            HeadingState.GAMEPAD)

        motors.forEach {
            it.motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        OpModeData.telemetry = telemetry
    }

    override fun onStartButtonPressed() {
        driverControlled = MecanumDriverControlledFixed(motors, gamepadManager.gamepad1.leftStick, headingSupplier::get, false, imu)
        driverControlled()


        // Gamepad 1
        gamepadManager.gamepad1.b.pressedCommand = {
            ParallelGroup(
                InstantCommand({ headingPID.target = Math.PI }),
                InstantCommand({ headingSupplier.state = HeadingState.PID })
            )
        }
        gamepadManager.gamepad1.x.pressedCommand = {
            ParallelGroup(
                InstantCommand({ headingPID.target = 0.0 }),
                InstantCommand({ headingSupplier.state = HeadingState.PID })
            )
        }
        gamepadManager.gamepad1.rightStick.displacedCommand = { InstantCommand({ headingSupplier.state = HeadingState.GAMEPAD }) }

        gamepadManager.gamepad1.rightBumper.pressedCommand =
            { InstantCommand({ driverControlled.scalar = 0.5 }) }
        gamepadManager.gamepad1.rightBumper.releasedCommand =
            { InstantCommand({ driverControlled.scalar = 1.0 }) }

        // Gamepad 2
        gamepadManager.gamepad2.dpadUp.pressedCommand = { Lift.toHigh }
        gamepadManager.gamepad2.dpadDown.pressedCommand = { Lift.toIntake }
        gamepadManager.gamepad2.rightTrigger.pressedCommand = { Lift.toTest }
        gamepadManager.gamepad2.leftTrigger.pressedCommand = { Lift.toClip }

        gamepadManager.gamepad2.y.pressedCommand = { HorizontalSlide.toOut }
        gamepadManager.gamepad2.a.pressedCommand = { HorizontalSlide.toIn }

        gamepadManager.gamepad2.rightBumper.pressedCommand = { ClawPivot.toDown }
        gamepadManager.gamepad2.leftBumper.pressedCommand = { ClawPivot.toUp }
    }
    override fun onUpdate() {
        telemetry.addData("Lift target position", Lift.controller.target)
        telemetry.addData("Lift current position", Lift.liftMotor.currentPosition)
        telemetry.addData("Slide target position", HorizontalSlide.controller.target)
        telemetry.addData("Slide current position", HorizontalSlide.slideMotor.currentPosition)
        telemetry.addData("Heading power", headingSupplier.get())
        telemetry.addData("Heading target", headingPID.target)
        telemetry.addData("Heading state", headingSupplier.state)
        telemetry.update()
    }
}