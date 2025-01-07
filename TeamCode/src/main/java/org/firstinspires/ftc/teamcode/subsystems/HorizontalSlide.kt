package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.rowanmcalpin.nextftc.core.Subsystem
import com.rowanmcalpin.nextftc.core.command.Command
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController
import com.rowanmcalpin.nextftc.ftc.OpModeData
import com.rowanmcalpin.nextftc.ftc.hardware.MotorToPosition

@Config
object HorizontalSlide: Subsystem() {

    lateinit var slideMotor: DcMotorEx

    @JvmField
    var kP: Double = 0.005

    @JvmField
    var kI: Double = 0.0

    @JvmField
    var kD: Double = 0.0

    @JvmField
    var kF: Double = 0.13

    val controller = PIDFController(kP, kI, kD, kF)

    @JvmField
    var attempted = false

    @JvmField
    var inPos = 0.0

    @JvmField
    var outPos = 100.0

    @JvmField
    var slideMotorName = "slide"

    val toIn: Command
        get() = MotorToPosition(slideMotor, inPos, controller, this)

    val toOut: Command
        get() = MotorToPosition(slideMotor, outPos, controller, this)

    override fun initialize() {
        slideMotor = OpModeData.hardwareMap.get(slideMotorName) as DcMotorEx
        slideMotor.direction = DcMotorSimple.Direction.REVERSE
        slideMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        slideMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }
}