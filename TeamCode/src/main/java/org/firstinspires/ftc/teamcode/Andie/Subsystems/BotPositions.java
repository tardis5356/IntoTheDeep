package org.firstinspires.ftc.teamcode.Andie.Subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class BotPositions {
    public static double GRIPPER_OPEN = .3, GRIPPER_CLOSED = 0.19, GRIPPER_INTAKE = .28;
    public static double WRIST_TRANSIT = 0.87, WRIST_WALL = .75, WRIST_INTAKE = .67, WRIST_BASKET = .46, WRIST_SPECIMEN = .75;
    public static double ARM_BASKET = .85, ARM_INTAKE = .323, ARM_SPECIMEN = .1, ARM_TRANSIT = .5, ARM_WALL =.1;
    public static double INTAKE_UP = 0.165, INTAKE_DOWN = 0.33, INTAKE_IN = 1, INTAKE_OUT = -1, INTAKE_STOP = 0;
    public static double EXTENDO_IN = 0.78, EXTENDO_OUT = .4, EXTENDO_MIDDLE =.7;
    public static int LIFT_WALL = -10, LIFT_SPECIMEN_HIGH = -10, LIFT_SPECIMEN_LOW = -10, LIFT_BASKET_HIGH = -500, LIFT_BASKET_LOW = -10, LIFT_TRANSIT = -500, LIFT_LIMIT = -3000;
    public static double RED_MAX = 400, RED_MIN = 300, BLUE_MAX = 150, BLUE_MIN = 110, YELLOW_MAX = 650, YELLOW_MIN = 620;
    public static double ANTI_GRAV = 0.1;
}
