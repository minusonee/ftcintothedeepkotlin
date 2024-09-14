package org.firstinspires.ftc.teamcode.drive.subsystems;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Config
@TeleOp
public class PIDF extends OpMode {
    private PIDController controller;

    public static double p = 0.025, i = 0, d = 0.0002;
    public static double f = 0.25;
    public static double delay = 0.004 ;

    public static int target = 0;

    private final double ticks_in_degree = 537.7 / 360;

    private DcMotorEx motorCraneLeft,motorCraneRight;

    @Override
    public void init(){
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        motorCraneLeft = hardwareMap.get(DcMotorEx.class, "motorCraneLeft");
        motorCraneRight = hardwareMap.get(DcMotorEx.class, "motorCraneRight");
        motorCraneLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorCraneRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorCraneLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorCraneRight.setDirection(DcMotorSimple.Direction.REVERSE);
        target = 40;
    }

    public void loop(){
        controller.setPID(p, i, d);
        int armPos = motorCraneLeft.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        if(gamepad1.right_trigger > 0.1){
            if (getRuntime() > delay) {
                this.resetRuntime();
                target++;
            }
        }
        if(gamepad1.left_trigger > 0.1){
            if (getRuntime() > delay) {
                this.resetRuntime();
                target--;
            }
        }
        motorCraneLeft.setPower(power);
        motorCraneRight.setPower(power);


        telemetry.addData("pos ", armPos);
        telemetry.addData("target ", target);
        telemetry.addData("Power ", power);
        telemetry.update();
    }
}
