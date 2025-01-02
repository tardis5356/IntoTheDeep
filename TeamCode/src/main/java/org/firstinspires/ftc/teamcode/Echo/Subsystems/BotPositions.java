package org.firstinspires.ftc.teamcode.Echo.Subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class BotPositions {
    public static double GRIPPER_OPEN = .6, GRIPPER_CLOSED = 0.36, GRIPPER_INTAKE = .475;
    public static double WRIST_TRANSIT = 0.87, WRIST_WALL = .92, WRIST_INTAKE = .39, WRIST_BASKET = .5, WRIST_SPECIMEN = .35;
    public static double ARM_BASKET = .85, ARM_INTAKE = .37, ARM_WALL = .2, ARM_TRANSIT = .5, ARM_SPECIMEN =.43, ARM_HANG = .61;
    public static double INTAKE_ARM_DOWN = 0.94, INTAKE_ARM_TRANSFER = 0.38, INTAKE_ARM_UP = .77, INTAKE_ARM_OUTAKE = .16,
            INTAKE_WRIST_DOWN = .95, INTAKE_WRIST_TRANSFER = .05, INTAKE_WRIST_UP = .8, INTAKE_WRIST_OUTAKE = .6,
            INTAKE_IN = 1, INTAKE_OUT = -1, INTAKE_STOP = 0, INTAKE_TRANSFER = 1;
    public static double EXTENDO_IN = 0.8, EXTENDO_OUT = .5, EXTENDO_SPECLEFT =.65, EXTENDO_SPECMID =.53, EXTENDO_SPECRIGHT = .5;

    //These are in motor ticks. On the ITD robot 100 ticks ~= 1 inch. Will differ from robot to robot.
    public static int LIFT_WALL = -200, LIFT_SPECIMEN_HIGH = -2000, LIFT_SPECIMEN_LOW = 0, LIFT_BASKET_HIGH = -3050,
            LIFT_BASKET_LOW = -890, LIFT_TRANSIT = -980, LIFT_LIMIT = -3100, LIFT_TOLERANCE = 10, LIFT_INTAKE = -470;
    public static double  INTAKE_RED_MIN = 200, INTAKE_BLUE_MIN = 190, YELLOW_MAX = 650, YELLOW_MIN = 620;
    public static double LIFT_FF = 0.1, WINCH_FF = 0;
    public static double LIFT_P = 0.007, LIFT_D = 0.0003, LIFT_I = 0.000;
}
