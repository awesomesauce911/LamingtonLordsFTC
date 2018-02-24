//package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class LamingtonLords extends OpMode

{
  private boolean v_warning_generated = false;
  private String v_warning_message;
  private DcMotor v_motor_left_drive;
  private DcMotor v_motor_right_drive;
  private DcMotor v_motor_basuri;
  private boolean first = true;
  private GyroSensor gyro;
  private OpticalDistanceSensor ODS;

  private ElapsedTime runtime = new ElapsedTime();

  byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable

  I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
  public static final int RANGE1_REG_START = 0x04; //Register to start reading
  public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read

  public I2cDevice RANGE1;
  public I2cDeviceSynch RANGE1Reader;


  private double TOLERANCE = 2; //Turn degree tolerance
  private int FORWARD_THRESHOLD_FOR_MAX_SPEED = 5;  //Threshold distance to use 1.0 speed, in turns

  private int DIST_FROM_START_TO_CAP_BALL = 150; //distances in cm
  private int DIST_FROM_BALL_TO_WALL = 150;

  float distance_for_one_rotation = 63.8f; //cm for 1 turn
  float drivePower = 1.0f;

  private int turnDist = 1380; //VALUE FOR A 90 DEG turn
  private int turnPower = 0.8; //SPEED FOR 90 DEG TURN

  int ONE_TILE_LENGTH = 160; //IN CM

  public LamingtonLords() {

  }

  @Override public void init()

  {

    v_warning_generated = false;
    v_warning_message = "Can't map; ";

    try
    {
      v_motor_left_drive = hardwareMap.dcMotor.get("Left Drive");
      v_motor_left_drive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

    }
    catch (Exception p_exeception)
    {
      m_warning_message("left_drive");
      DbgLog.msg(p_exeception.getLocalizedMessage());

      v_motor_left_drive = null;
    }

    try
    {
      v_motor_right_drive = hardwareMap.dcMotor.get("Right Drive");
      v_motor_right_drive.setDirection(DcMotor.Direction.REVERSE);
      v_motor_right_drive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }
    catch (Exception p_exeception)
    {
      m_warning_message("right_drive");
      DbgLog.msg(p_exeception.getLocalizedMessage());

      v_motor_right_drive = null;
    }

    try
    {
      v_motor_basuri = hardwareMap.dcMotor.get("Basuri");
      v_motor_basuri.setDirection(DcMotor.Direction.REVERSE);
    }
    catch (Exception p_exeception)
    {
      m_warning_message("basuri");
      DbgLog.msg(p_exeception.getLocalizedMessage());

      v_motor_right_drive = null;
    }

    try
    {
      gyro = hardwareMap.gyroSensor.get("gyro");

    }
    catch (Exception p_exeception)
    {
      m_warning_message("gyro");
      DbgLog.msg(p_exeception.getLocalizedMessage());

      gyro = null;
    }
  }
  @Override public void loop()

  {
    //DRIVE USING STICKS
    v_motor_left_drive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    v_motor_right_drive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    float left_drive_power = (-gamepad1.left_stick_y + gamepad1.left_stick_x) / 2;
    float right_drive_power = (-gamepad1.left_stick_y - gamepad1.left_stick_x) / 2;
    setDrivePower(left_drive_power, right_drive_power);

    //HANDBRAKE
    if (gamepad1.right_bumper)
    {
    setDrivePower(0.0 f, 0.0 f);
    v_motor_right_drive.setTargetPosition(v_motor_right_drive.getCurrentPosition());
    v_motor_left_drive.setTargetPosition(v_motor_left_drive.getCurrentPosition());
    v_motor_left_drive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
    v_motor_right_drive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

    }

    //x BUTTON TOGGLES BASURI
    if (gamepad1.x && (v_motor_basuri.getPower() == 0.0))
    v_motor_basuri.setPower(1.0f);
    else if (gamepad1.x && (v_motor_basuri.getPower() == 1.0))
    v_motor_basuri.setPower(0.0f);


    //LEFT ON DPAD TURNS LEFT, RIGHT TURNS RIGHT
    if(gamepad1.dpad_left)
    turnLeft();

    if(gamepad1.dpad_right)
    turnRight();

    //UP ON D_PAD MOVES ONE TILE
    if(gamepad1.dpad_up)
    moveForward(ONE_TILE_LENGTH);


    update_telemetry();
    update_gamepad_telemetry();

  }
  void turnLeft()
  {
    if (!gyro.isCalibrating())
    {
      double target_angle_degrees = 90;
      double error_degrees = Math.abs(target_angle_degrees - gyro.getHeading());
      double motor_output = (error_degrees / 180);

      if(error_degrees > TURN_ERROR_THRESHOLD)
      {
        setDrivePower(1.0f,1.0f);
      }
      else if (error_degrees < TOLERANCE)
      {
        setDrivePower(MOTOR_MINIMUM_TURN_POWER,MOTOR_MINIMUM_TURN_POWER);
      }
      else
      {
        setDrivePower(motor_output, -motor_output);
      }
    }

  }

  void turnRight()
  {
    if (!gyro.isCalibrating())
    {
      double target_angle_degrees = 90;
      double error_degrees = Math.abs(target_angle_degrees - gyro.getHeading());
      double motor_output = (error_degrees / 180);

      if(error_degrees > TURN_ERROR_THRESHOLD)
      {
        setDrivePower(1.0f,1.0f);
      }
      else if (error_degrees < TOLERANCE)
      {
        setDrivePower(MOTOR_MINIMUM_TURN_POWER,MOTOR_MINIMUM_TURN_POWER);
      }
      else
      {
        setDrivePower(-motor_output, motor_output);
      }
    }

  }

  public void update_telemetry()

  {
    telemetry.addData("01", "Left Drive: " + getPower(v_motor_left_drive));
    telemetry.addData("02", "Right Drive: " + getPower(v_motor_right_drive));
    telemetry.addData("03", "Basuri: " + getPower(v_motor_basuri));

  }

  public void update_gamepad_telemetry()

  {

    telemetry.addData("04", "GP1 Left Stick Y: " + gamepad1.left_stick_y);
    telemetry.addData("05", "GP1 Right Stick Y: " + gamepad1.right_stick_y);
    telemetry.addData("06", "GP1 Left Stick X: " + gamepad1.left_stick_x);
    telemetry.addData("07", "GP1 Right Stick X: " + gamepad1.right_stick_x);
    telemetry.addData("08", "GP1 LT: " + gamepad1.left_trigger);
    telemetry.addData("09", "GP1 RT: " + gamepad1.right_trigger);

  }


  double getPower(DcMotor x) {
    double l_return = 0.0;

    if (x != null) {
      l_return = x.getPower();
    }

    return l_return;

  }

  void setDrivePower(float a, float b) {
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

}
