package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Filtered RobotCore Logs", group = "Examples")
public class FilteredRobotCoreLogsOpMode extends LinearOpMode {

    private FtcDashboard dashboard;
    private Process logcatProcess;
    private BufferedReader bufferedReader;
    private List<String> logEntries;

    @Override
    public void runOpMode() {
        // Initialize the FTC Dashboard
        dashboard = FtcDashboard.getInstance();
        logEntries = new ArrayList<>();

        // Start capturing Logcat output
        startLogcatCapture();

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            try {
                // Read new lines from Logcat output
                while (bufferedReader != null && bufferedReader.ready()) {
                    String logEntry = bufferedReader.readLine();
                    if (logEntry != null && logEntry.contains("RobotCore")) {
                        // Add filtered log entry to the list
                        logEntries.add(logEntry);
                    }
                }

                // Send new filtered log entries to the dashboard if there are any
                if (!logEntries.isEmpty()) {
                    sendLogsToDashboard(logEntries);
                    logEntries.clear(); // Clear the list after sending
                }
            } catch (IOException e) {
                telemetry.addData("Error", "Failed to read Logcat: " + e.getMessage());
                telemetry.update();
            }

            // Sleep for a short duration to control the update rate
            sleep(100);
        }

        // Stop capturing Logcat output when done
        stopLogcatCapture();
    }

    /**
     * Starts capturing Logcat output.
     */
    private void startLogcatCapture() {
        try {
            // Start the logcat process
            logcatProcess = Runtime.getRuntime().exec("logcat");
            bufferedReader = new BufferedReader(new InputStreamReader(logcatProcess.getInputStream()));
        } catch (IOException e) {
            telemetry.addData("Error", "Failed to start Logcat capture: " + e.getMessage());
            telemetry.update();
        }
    }

    /**
     * Stops capturing Logcat output.
     */
    private void stopLogcatCapture() {
        if (logcatProcess != null) {
            logcatProcess.destroy();
        }
    }

    /**
     * Sends new filtered log entries to the FTC Dashboard.
     *
     * @param logs the new filtered log entries to send
     */
    private void sendLogsToDashboard(List<String> logs) {
        TelemetryPacket packet = new TelemetryPacket();
        for (int i = 0; i < logs.size(); i++) {
            packet.put("Log Entry " + i, logs.get(i));
        }
        dashboard.sendTelemetryPacket(packet);

        // Optionally, also update the driver station telemetry
        for (int i = 0; i < logs.size(); i++) {
            telemetry.addData("Log Entry " + i, logs.get(i));
        }
        telemetry.update();
    }
}