/*
                _                                              __  __           _      
     /\        | |                                            |  \/  |         | |     
    /  \  _   _| |_ ___  _ __   ___  _ __ ___   ___  _   _ ___| \  / | ___   __| | ___ 
   / /\ \| | | | __/ _ \| '_ \ / _ \| '_ ` _ \ / _ \| | | / __| |\/| |/ _ \ / _` |/ _ \
  / ____ \ |_| | || (_) | | | | (_) | | | | | | (_) | |_| \__ \ |  | | (_) | (_| |  __/
 /_/    \_\__,_|\__\___/|_| |_|\___/|_| |_| |_|\___/ \__,_|___/_|  |_|\___/ \__,_|\___|
                                                                                       
*/


package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.lang.*;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name = "AutonomousMode04", group = "Sensor")

// @Disabled
public class AutonomousMode03 extends LinearOpMode {
    private DistanceSensor sensorRangeR;
    private DistanceSensor sensorRangeL;

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor motorR, motorL, motorLift;
    double leftPower, rightPower;
    Servo hookLock, servo4;
    double t = 0;
    double error = 3;
    Rev2mDistanceSensor odsR;
    Rev2mDistanceSensor odsL;


    @Override
    public void runOpMode() {
        // Distance Sensors
        odsR = (Rev2mDistanceSensor) hardwareMap.get(DistanceSensor.class, "odsR");
        odsL = (Rev2mDistanceSensor) hardwareMap.get(DistanceSensor.class, "odsL");

        // Motors
        motorR = hardwareMap.get(DcMotor.class, "right_drive");
        motorL = hardwareMap.get(DcMotor.class, "left_drive");
        motorL.setDirection(DcMotorSimple.Direction.REVERSE); // revesed b/c of mirror
        motorLift = hardwareMap.get(DcMotor.class, "motor_lift");

        // Servos
            //hookLock = hardwareMap.get(Servo.class, "servo3");
        servo4 = hardwareMap.get(Servo.class, "servo4");
        // Servo Position Setting
            //hookLock.setPosition(.9);
        servo4.setPosition(0);

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRangeR;
        telemetry.addData(">>", "Press start to continue");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()) {

            // Generic DistanceSensor Methods
            /*hookLock.setPosition(0);
            if(t < 200) {

                // Wiggling Movement
                sleep(10);
                t = t + 1;
                rightPower = (Math.cos(Math.PI * 2 * t)) / 2;
                leftPower = -rightPower;
            }
                motorR.setPower(Range.clip(-leftPower, -1.0, 1.0));
                motorL.setPower(Range.clip(-rightPower, -1.0, 1.0));
                motorR.setPower(-1);
                motorL.setPower(-1);
                sleep(500);
                motorR.setPower(1);
                motorL.setPower(-1);
                sleep(700);
            */
           
           
            // Elevator Code
            if (t < 200) {
                
                // Release Mechanism
                t = t + 1;
                motorLift.setPower(-1);
                sleep(700);

            }

            // Marker Placement
            leftPower = .4;
            rightPower = .4;
            if(sensorRangeR.getDistance(DistanceUnit.CM) - sensorRangeL.getDistance(DistanceUnit.CM) > error);
            rightPower = rightPower-0.4;
            if(sensorRangeL.getDistance(DistanceUnit.CM) - sensorRangeR.getDistance(DistanceUnit.CM) > error);
            leftPower = leftPower-0.4;
            motorL.setPower(leftPower);
            motorR.setPower(rightPower);
            sleep(500);
            motorR.setPower(Range.clip(-leftPower, -1.0, 1.0));
            motorL.setPower(Range.clip(-rightPower, -1.0, 1.0));
            if(sensorRangeR.getDistance(DistanceUnit.CM) <= 10);
            rightPower = 0;
            if(sensorRangeL.getDistance(DistanceUnit.CM) <= 10);
            leftPower = 0;
            motorL.setPower(leftPower);
            motorR.setPower(rightPower);
            if(leftPower + rightPower <= 0.01);
            servo4.setPosition(.7);
            telemetry.addData("rangeR", String.format("%.01f cm", sensorRangeR.getDistance(DistanceUnit.CM)));
            telemetry.addData("rangeL", String.format("%.01f cm", sensorRangeL.getDistance(DistanceUnit.CM)));

            // Rev2mDistanceSensor Specific Methods
            telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));
            telemetry.update();
        }
    }
}
