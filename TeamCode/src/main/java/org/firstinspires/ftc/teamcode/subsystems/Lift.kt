package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.rowanmcalpin.nextftc.core.Subsystem
import com.rowanmcalpin.nextftc.core.command.Command
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward
import com.rowanmcalpin.nextftc.ftc.OpModeData
import com.rowanmcalpin.nextftc.ftc.hardware.MotorToPosition

@Config
object Lift: Subsystem() {

    lateinit var liftMotor: DcMotorEx

    @JvmField
    var kP: Double = 0.005

    @JvmField
    var kI: Double = 0.0

    @JvmField
    var kD: Double = 0.0

    @JvmField
    var kF: Double = 0.13

    val controller = PIDFController(kP, kI, kD, StaticFeedforward(kF))

    @JvmField
    var attempted = false

    @JvmField
    var intakePos = 0.0

    @JvmField
    var testPos = 300.0

    @JvmField
    var clipPos = 1012.0

    @JvmField
    var highBarPos = 1380.0

    @JvmField
    var liftMotorName = "lift"

    val toIntake: Command
        get() = MotorToPosition(liftMotor, intakePos, controller, this)

    val toTest: Command
        get() = MotorToPosition(liftMotor, testPos, controller, this)

    val toClip: Command
        get() = MotorToPosition(liftMotor, clipPos, controller, this)

    val toHigh: Command
        get() = MotorToPosition(liftMotor, highBarPos, controller, this)

    override fun initialize() {
        liftMotor = OpModeData.hardwareMap.get(liftMotorName) as DcMotorEx
        liftMotor.direction = DcMotorSimple.Direction.REVERSE
        liftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        liftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }
}