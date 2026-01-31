import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name = "AutoTest")


public class Autotest extends LinearOpMode {
    private DcMotor fLeft;
    private DcMotor fRight;
    private DcMotor bLeft;
    private DcMotor bRight;

    private int fLeftPos;
    private int bLeftPos;
    private int fRightPos;
    private int bRightPos;
    private ElapsedTime stopwatch = new ElapsedTime();


    public void runOpMode() {
        fLeft = hardwareMap.get(DcMotor.class, "front_left_drive");
        fRight = hardwareMap.get(DcMotor.class, "front_right_drive");
        bLeft = hardwareMap.get(DcMotor.class, "back_left_drive");
        bRight = hardwareMap.get(DcMotor.class, "back_right_drive");

        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //19.4 encoders/foot

        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        fLeftPos = 0;
        fRightPos = 0;
        bLeftPos = 0;
        bRightPos = 0;

        waitForStart();
        //funny dave mode
        //drive(1000, -1000, .25);
        drive(600, 600, .3);
    }

    private void drive(int lTar, int rTar, double speed) {
        stopwatch.reset();
        bRightPos += rTar;
        fRightPos += rTar;

        bLeftPos += lTar;
        fLeftPos += lTar;

        fLeft.setTargetPosition(fLeftPos);
        bLeft.setTargetPosition(bLeftPos);
        fRight.setTargetPosition(fRightPos);
        bRight.setTargetPosition(bRightPos);

        fLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fLeft.setPower(speed);
        bLeft.setPower(speed);
        fRight.setPower(speed);
        bRight.setPower(speed);

        while (stopwatch.time() <0.5) {

        }
        fLeft.setPower(0);
        bLeft.setPower(0);
        fRight.setPower(0);
        bRight.setPower(0);

    }
}