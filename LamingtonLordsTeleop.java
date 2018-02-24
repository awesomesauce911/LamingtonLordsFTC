package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.*;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;
import java.util.*;
import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.provider.MediaStore;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.content.pm.PackageManager;

import java.io.File;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.content.pm.ResolveInfo;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.net.Uri;
import android.os.Build;
import android.os.Bundle;
import android.os.Environment;
import android.provider.MediaStore;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.VideoView;

@TeleOp(name="Lamington Lords Teleop", group="Final Day")

public class LamingtonLordsTeleop extends OpMode

{
    private boolean v_warning_generated = false;
    private String v_warning_message;
    private DcMotor v_motor_left_drive;
    private DcMotor v_motor_right_drive;
    private DcMotor flywheel1;
    private DcMotor flywheel2;
    private DcMotor v_motor_basuri;
    private DcMotor v_motor_conveyor;
    private boolean first = true;
    private GyroSensor gyro;
 //
 // private Servo beacon1;
  private Servo beacon2;
    private OpticalDistanceSensor ODS;

  //  private ModernRoboticsI2cColorSensor cs;

    private ElapsedTime runtime = new ElapsedTime();

    //VARIABLES TO CALIBRATE
    final float TURN_ERROR_THRESHOLD = 10; //MINIMUM DEGREES OF ERROR AFTER WHICH MINIMUM POWER STARTS
    final double TOLERANCE =3; //DEGREES OF TURN ERROR WHICH IS CLOSE ENOUGH
    final float MOTOR_MINIMUM_TURN_POWER = 0.2f;
    final int FORWARD_THRESHOLD_FOR_MAX_SPEED = 5;  //Threshold distance to use 1.0 speed, in turns

    final float MOTOR_MAX_POWER = 1.0f;
    final int TICKS_PER_TURN = 1440;
    final int DIST_FROM_CAP_BALL_TO_BEACON2 = 30;
    final int RADIUS_OF_WHEEL_IN = 2; //inches
    final double DISTANCE_PER_TURN = RADIUS_OF_WHEEL_IN*2.54*2*Math.PI/4;
    int DIST_TOLERANCE = 5;//MIN DIST in cm BEFORE MIN POWER
    double STICK_THRESHOLD = 0.2; //STICK VALUE AFTER WHICH SERVO BEGINS TO TURN
    double SERVO_ANGLE_RATE = 5; //DEGREES / SECOND OF ROTATION OF THE SERVO
    int FLYWHEEL_SERVO_MAX_POS = 50; //DEGREES MAX POSITION OF FLYWHEEL SERVO
    int FLYWHEEL_SERVO_MIN_POS = 20; //DEGREES MIN POSITION OF FLYWHEEL SERVO
    float FLYWHEEL_SPEED_RATE = 0.2f;
    final float TURN_POWER = 0.5f;
    int ONE_TILE_LENGTH = 160; //IN CM


    private ElapsedTime time_since_b_press = new ElapsedTime();
    private ElapsedTime time_since_x_press = new ElapsedTime();
    private ElapsedTime time_since_a_press = new ElapsedTime();

    private ElapsedTime time_since_lt1_press = new ElapsedTime();
    private ElapsedTime time_since_rt1_press = new ElapsedTime();


    final double TURN_SPEED = 0.8f;


    public LamingtonLordsTeleop() {

    }

    @Override public void init()

    {



        //beacon1 = hardwareMap.servo.get("Beacon Left");
        beacon2= hardwareMap.servo.get("Beacon Right");
          //if(beacon1!=null)
          //    beacon1.setPosition(0.0);
        if(beacon2!=null)
            beacon2.setPosition(0.0);


        time_since_x_press.startTime();
        time_since_b_press.startTime();
        time_since_a_press.startTime();
        time_since_lt1_press.startTime();
        time_since_rt1_press.startTime();

        v_warning_generated = false;
        v_warning_message = "Can't map; ";


       /** try {
            beacon1 = hardwareMap.servo.get("Beacon Left");
            beacon2 = hardwareMap.servo.get("Beacon Right");
            beacon1.setPosition(0.0f);
            beacon2.setPosition(0.0f);
        } catch (Exception e) {
            m_warning_message("Beacon servos");
            DbgLog.msg(e.getLocalizedMessage());
        }**/

        try {
            flywheel1 = hardwareMap.dcMotor.get("Flywheel 1");
            flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel1.setDirection(DcMotor.Direction.REVERSE);

        } catch (Exception e) {
            m_warning_message("Flywheel 1");
            DbgLog.msg(e.getLocalizedMessage());

            flywheel1 = null;
        }

        try {
            flywheel2 = hardwareMap.dcMotor.get("Flywheel 2");
            flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        } catch (Exception e) {
            m_warning_message("Flywheel 2");
            DbgLog.msg(e.getLocalizedMessage());

            flywheel2 = null;
        }

        try {
            v_motor_left_drive = hardwareMap.dcMotor.get("Left Drive");
            v_motor_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            v_motor_left_drive.setDirection(DcMotor.Direction.REVERSE);


        } catch (Exception e) {
            m_warning_message("left_drive");
            DbgLog.msg(e.getLocalizedMessage());

            v_motor_left_drive = null;
        }

        try {
            v_motor_right_drive = hardwareMap.dcMotor.get("Right Drive");
            v_motor_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            m_warning_message("right_drive");
            DbgLog.msg(e.getLocalizedMessage());

            v_motor_right_drive = null;
        }


        try {
            v_motor_basuri = hardwareMap.dcMotor.get("Basuri");
            v_motor_basuri.setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception e) {
            m_warning_message("basuri");
            DbgLog.msg(e.getLocalizedMessage());

            v_motor_basuri = null;
        }


        try {
            v_motor_conveyor = hardwareMap.dcMotor.get("Conveyor");
            // v_motor_conveyor.setDirection(DcMotor.Direction.REVERSE);

        } catch (Exception e) {
            m_warning_message("conveyor");
            DbgLog.msg(e.getLocalizedMessage());

            v_motor_conveyor = null;
        }


        try {
            gyro = hardwareMap.gyroSensor.get("Gyro");
            if (gyro != null)
                calibrateGyro();

        } catch (Exception e) {
            m_warning_message("gyro");
            DbgLog.msg(e.getLocalizedMessage());

            gyro = null;
        }
    }


    @Override public void loop()

    {


        //DRIVING CONTROLLS - GAMEPAD 1
        //-----------------------------
        //DRIVE USING STICKS
        v_motor_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        v_motor_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        float left_drive_power =  Range.clip(((-gamepad1.left_stick_y -
                gamepad1.left_stick_x)),-1,1);
        float right_drive_power =  Range.clip(((-gamepad1.left_stick_y +
                gamepad1.left_stick_x) ),-1,1);
        if (left_drive_power != v_motor_left_drive.getPower() || right_drive_power != v_motor_right_drive.getPower()) {
            setDrivePower(left_drive_power,right_drive_power);
        }


        //HANDBRAKE
        if (gamepad1.right_bumper)
        {
            setDrivePower(0.0f, 0.0f);
            v_motor_right_drive.setTargetPosition(v_motor_right_drive.getCurrentPosition());
            v_motor_left_drive.setTargetPosition(v_motor_left_drive.getCurrentPosition());
            v_motor_left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            v_motor_right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

        //LEFT ON DPAD TURNS LEFT, RIGHT TURNS RIGHT
        if(gamepad1.dpad_left) {
            try {
                turnLeft();
            }
            catch (Exception e)
            {
                DbgLog.msg("Turn left interrupt "+e.getLocalizedMessage());
            }

        }

        if(gamepad1.dpad_right)
            try {
                turnRight();
            }
            catch (Exception e)
            {
                DbgLog.msg("Turn left interrupt "+e.getLocalizedMessage());
            }

        //UP ON D_PAD MOVES ONE TILE
        if(gamepad1.dpad_up)
            try {
                moveForward(ONE_TILE_LENGTH);
            }
            catch(Exception e)
            {
                DbgLog.msg(e.getLocalizedMessage());
            }

        //DOWN MOVE BACK ONE TILE
        if(gamepad1.dpad_down)
            try {
                moveBackward(ONE_TILE_LENGTH);
            }
            catch(Exception e)
            {
                DbgLog.msg(e.getLocalizedMessage());
            }

        if(time_since_x_press.seconds()>0.5) {
            if (gamepad1.x && (v_motor_basuri.getPower() == 0.0))

            {
                v_motor_basuri.setPower(1.0f);
                time_since_x_press.reset();
            } else if (gamepad1.x && (v_motor_basuri.getPower() == 1.0)) {
                v_motor_basuri.setPower(0.0f);
                time_since_x_press.reset();
            }
        }





        if(time_since_rt1_press.seconds()>0.5) {
            if (gamepad1.right_trigger>0.1 && (beacon2.getPosition() ==0))

            {
                beacon2.setPosition(1);
                time_since_rt1_press.reset();

            } else if (gamepad1.right_trigger>0.1 && (beacon2.getPosition() ==1)) {
                beacon2.setPosition(0);
                time_since_rt1_press.reset();
            }
        }


        //SHOOTING CONTROLS - GAMEPAD 2
        //-----------------------------
        //a BUTTON TOGGLES CONVEYOPR

        if(time_since_a_press.seconds()>0.5) {
            if (gamepad2.a && (v_motor_conveyor.getPower() == 0.0))

            {
                v_motor_conveyor.setPower(1.0f);
                v_motor_conveyor.setDirection(DcMotor.Direction.FORWARD);
                time_since_a_press.reset();
            } else if (gamepad2.a && (v_motor_conveyor.getPower() == 1.0)) {
                v_motor_conveyor.setPower(0.0f);
                v_motor_conveyor.setDirection(DcMotor.Direction.FORWARD);
                time_since_a_press.reset();
            }
        }

        //x BUTTON TOGGLES CONVEYOR DIRECTION
        if(time_since_b_press.seconds()>0.5) {
            if (gamepad2.x && (v_motor_conveyor.getDirection().equals(DcMotor.Direction.FORWARD))) {
                v_motor_conveyor.setDirection(DcMotor.Direction.REVERSE);
                time_since_b_press.reset();

            } else if (gamepad2.x && (v_motor_conveyor.getDirection().equals(DcMotor.Direction.REVERSE))){
                v_motor_conveyor.setDirection(DcMotor.Direction.FORWARD);
                time_since_b_press.reset();

            }
        }


        //GAMEPAD 2 LEFT STICK SPEED CONTROL
        double flywheel_speed = Range.clip(-gamepad2.left_stick_y,0.0f,1.0f);
        flywheel1.setPower(flywheel_speed);
        flywheel2.setPower(flywheel_speed);


        telemetry.addData("Gyro: ","Heading:"+gyro.getHeading());
        update_telemetry();






    }

    void turnLeft() throws InterruptedException {


        int currentHeading = gyro.getHeading();
        int target_angle_degrees = (currentHeading + 270) % 360;

        double lb = target_angle_degrees-TOLERANCE;
        if(lb<0)
            lb=360+lb;
        double ub = target_angle_degrees+TOLERANCE;
        if(ub>360)
            ub=ub%360;

        while (!(gyro.getHeading() < ub && gyro.getHeading() > lb)) {
            int error = Math.abs(target_angle_degrees - gyro.getHeading());
            if (error > TURN_ERROR_THRESHOLD)
                setDrivePower(TURN_POWER, -TURN_POWER);
            else
                setDrivePower(MOTOR_MINIMUM_TURN_POWER, -MOTOR_MINIMUM_TURN_POWER);

            telemetry.addData("Gyro: ", "Heading:" + gyro.getHeading() + " Error: " + error);
            telemetry.update();
        }
        setDrivePower(0,0);

    }

    void turnRight() throws InterruptedException {


        int currentHeading = gyro.getHeading();
        int target_angle_degrees = (currentHeading + 90) % 360;

        double lb = target_angle_degrees-TOLERANCE;
        if(lb<0)
            lb=360+lb;
        double ub = target_angle_degrees+TOLERANCE;
        if(ub>360)
            ub=ub%360;

        while (!(gyro.getHeading() < ub && gyro.getHeading() > lb)) {
            int error = Math.abs(target_angle_degrees - gyro.getHeading());
            if (error > TURN_ERROR_THRESHOLD)
                setDrivePower(-TURN_POWER, TURN_POWER);
            else
                setDrivePower(-MOTOR_MINIMUM_TURN_POWER, MOTOR_MINIMUM_TURN_POWER);

            telemetry.addData("Gyro: ", "Heading:" + gyro.getHeading() + " Error: " + error);
            telemetry.update();
        }
        setDrivePower(0,0);

    }



    public void moveForward(double distance) throws InterruptedException {


        int turns = (int) (distance / DISTANCE_PER_TURN);
        v_motor_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Resets encoders
        v_motor_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(v_motor_left_drive.getCurrentPosition() != 0) { //Ensures encoders are zero
            v_motor_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            v_motor_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }


        v_motor_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Sets mode to use encoders
        v_motor_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //setMode() is used instead of setChannelMode(), which is now deprecated

        int targetPos = TICKS_PER_TURN*turns;
        v_motor_left_drive.setTargetPosition(targetPos); //Sets motor to move 1440 ticks (1440 is one rotation for Tetrix motors)
        v_motor_right_drive.setTargetPosition(targetPos);
        int lPos = v_motor_left_drive.getCurrentPosition();
        int rPos = v_motor_right_drive.getCurrentPosition();

        int lError = Math.abs(lPos-targetPos);
        int rError = Math.abs(rPos-targetPos);

        int inlError = lError;
        int inrError = rError;

        while(v_motor_left_drive.getCurrentPosition() < v_motor_left_drive.getTargetPosition() || v_motor_right_drive.getCurrentPosition() < v_motor_right_drive.getTargetPosition()) { //While target has not been reached



            if(lError>FORWARD_THRESHOLD_FOR_MAX_SPEED &&
                    rError>FORWARD_THRESHOLD_FOR_MAX_SPEED)
            {

                setDrivePower(MOTOR_MAX_POWER,MOTOR_MAX_POWER);
            }

            else if(lError< DIST_TOLERANCE && rError < DIST_TOLERANCE)
            {

                setDrivePower(MOTOR_MINIMUM_TURN_POWER,MOTOR_MINIMUM_TURN_POWER);
            }
            else
            {

                setDrivePower(lError/inlError,rError/inrError);
            }

            lPos = v_motor_left_drive.getCurrentPosition();
            rPos = v_motor_right_drive.getCurrentPosition();

            lError = Math.abs(lPos-targetPos);
            rError = Math.abs(rPos-targetPos);


        }

        v_motor_left_drive.setPower(0);
        v_motor_right_drive.setPower(0);
    }

    //void makeCamera()




   /** void turnRightServo() throws InterruptedException
    {
        beacon2.setPosition(1.0f);

        Thread.sleep(3);

        beacon2.setPosition(0.0f);
    }
    void turnLeftServo() throws InterruptedException
    {
        beacon1.setPosition(1.0f);
        Thread.sleep(3);
        beacon1.setPosition(0.0f);
    }**/

   public void moveBackward(double distance) throws InterruptedException {



       flipDrive();

       moveForward(distance);
       flipDrive();

   }


    void flipDrive()
    {
        if(v_motor_left_drive.getDirection().equals(DcMotor.Direction.FORWARD))
        {
            v_motor_left_drive.setDirection(DcMotor.Direction.REVERSE);
        }
        else
        {
            v_motor_left_drive.setDirection(DcMotor.Direction.FORWARD);
        }

        if(v_motor_right_drive.getDirection().equals(DcMotor.Direction.FORWARD))
        {
            v_motor_right_drive.setDirection(DcMotor.Direction.REVERSE);
        }
        else
        {
            v_motor_right_drive.setDirection(DcMotor.Direction.FORWARD);
        }


    }

    public void update_telemetry()

    {
        telemetry.addData("00", "Gyro head: " + gyro.getHeading());
        telemetry.addData("01", "Left Drive: " + getPower(v_motor_left_drive));
        telemetry.addData("02", "Right Drive: " + getPower(v_motor_right_drive));
        telemetry.addData("03", "Basuri: " + getPower(v_motor_basuri));
        telemetry.addData("04", "Conveyor: " + getPower(v_motor_conveyor));
        telemetry.addData("05", "Flywheels: " + getPower(flywheel1) + "," + getPower(flywheel2));
      //  telemetry.addData("06", "Beacon Servos: "+beacon1.getPosition()+","+beacon2.getPosition());
       // telemetry.addData("07", "Color: R="+cs.red()+" ,ARGB:"+cs.argb());

        telemetry.update();

    }




    double getPower(DcMotor x) {
        double l_return = 0.0;

        if (x != null) {
            l_return = x.getPower();
        }

        return l_return;

    }

    void setDrivePower(double a, double b) {
        v_motor_left_drive.setPower(a);
        v_motor_right_drive.setPower(b);
    }

    void m_warning_message(String p_exception_message)
    {
        if (v_warning_generated) {
            v_warning_message += ", ";
        }
        v_warning_generated = true;
        v_warning_message += p_exception_message;

    }
    void calibrateGyro () throws InterruptedException {



        gyro.calibrate();

        while (gyro.isCalibrating()) {
            telemetry.addData("Calibrated", false);
            telemetry.update();
        }

        telemetry.addData("Calibrated", true);
        telemetry.update();
    }

}
