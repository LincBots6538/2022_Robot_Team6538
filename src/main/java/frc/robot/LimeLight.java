// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/** Returns the values for the LimeLight Sensor */
public class LimeLight {

private static double x;
private static double area;

public void UpdateLimeLight(){

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    //read values periodically
    x = tx.getDouble(0);
    double y = ty.getDouble(0);
    area = ta.getDouble(0);

    //post to smart DashBoard Periodically
    SmartDashboard.putNumber("LimelightX" , x);
    SmartDashboard.putNumber("LimelightY" , y);
    SmartDashboard.putNumber("LimelightArea" , area);

}
public static double getx(){

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");

return tx.getDouble(0);
}

public static double getTargetx() {

    return x;
}

/* returns the area of the bounding square */
public static double getTargetArea() {

    return area;
}

}
