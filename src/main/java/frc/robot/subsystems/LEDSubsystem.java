package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import robowled.wledpipe.DummyPipe;
import robowled.wledpipe.SerialPipe;
import robowled.wledpipe.WledPipe;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private WledPipe wled;

    public LEDSubsystem() {
        if (RobotBase.isSimulation()) {
            wled = new DummyPipe(msg -> System.out.println("[WLED] " + msg.trim()));
        } else {
            try {
                wled = new SerialPipe(SerialPort.Port.kUSB, 115200);
            } catch (Exception e) {
                System.err.println("Failed to connect to WLED, using dummy");
                wled = new DummyPipe();
            }
        }
    }

    public void setAllianceColor() {
        try {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                if (alliance.get() == DriverStation.Alliance.Red) {
                    // Red alliance - set LEDs to red
                    wled.sendString("{\"seg\":[{\"col\":[[255,0,0]]}]}\n");
                } else {
                    // Blue alliance - set LEDs to blue
                    wled.sendString("{\"seg\":[{\"col\":[[0,0,255]]}]}\n");
                }
            }
        } catch (Exception e) {
            System.err.println("Failed to set alliance color: " + e.getMessage());
        }
    }

    public void setEnabled() {
        try {
            wled.sendString("{\"on\":true,\"bri\":255}\n");
        } catch (Exception e) {
            System.err.println("Failed to enable LEDs: " + e.getMessage());
        }
    }

    public void setDisabled() {
        try {
            wled.sendString("{\"on\":true,\"bri\":50}\n");
        } catch (Exception e) {
            System.err.println("Failed to dim LEDs: " + e.getMessage());
        }
    }
}