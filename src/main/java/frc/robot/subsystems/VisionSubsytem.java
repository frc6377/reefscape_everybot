//(x, y, z) and orientation (pitch, roll, yaw) relative to the camera.
package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsytem extends SubsystemBase {

  private NetworkTableEntry m_botPos;
  private NetworkTableEntry m_camPos;


  public VisionSubsytem() {
    m_botPos = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose");
    m_camPos = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace");
  }

  @Override
  public void periodic() {
    double[] botPose = m_botPos.getDoubleArray(new double[6]);
    double[] camPose = m_camPos.getDoubleArray(new double[6]);

    if (botPose.length != 0) {
      SmartDashboard.putNumber("x bot pose", botPose[0]);
      SmartDashboard.putNumber("y bot pose", botPose[1]);
      SmartDashboard.putNumber("z bot pose", botPose[2]);
    }

    if (camPose.length != 0) {
      SmartDashboard.putNumber("x tag pose", camPose[0]);
      SmartDashboard.putNumber("y tag  pose (Height): ", camPose[1]);
      SmartDashboard.putNumber("z tag pose (Angle): ", camPose[2]);


    // write code to move the robot below here after odometry works for kitbot
    }
  }
}