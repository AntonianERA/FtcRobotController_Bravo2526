import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name="outtake test", group="Linear OpMode")
public class outtake extends LinearOpMode{

    private DcMotor leftOutTake;
    private DcMotor rightOutTake;

    @Override
    public void runOpMode(){
        rightOutTake = hardwareMap.get(DcMotor.class, "right_outtake");
        leftOutTake = hardwareMap.get(DcMotor.class, "left_outtake");
        while(opModeIsActive()){
            leftOutTake.setPower(1);
            rightOutTake.setPower(1);
        }
    }


}
