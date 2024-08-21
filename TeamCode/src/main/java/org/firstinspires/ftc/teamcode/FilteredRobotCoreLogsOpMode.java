package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;

@TeleOp(name = "Filtered RobotCore Logs", group = "Examples")
public class FilteredRobotCoreLogsOpMode extends LinearOpMode {

    private FtcDashboard dashboard;
    private Process logcatProcess;
    private BufferedReader bufferedReader;

    @Override
    public void runOpMode() {
        // Initialize the FTC Dashboard
        dashboard = FtcDashboard.getInstance();

        // Start capturing Logcat output
        startLogcatCapture();

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            try {
                // Read new lines from Logcat output
                StringBuilder newLogs = new StringBuilder();
                while (bufferedReader != null && bufferedReader.ready()) {
                    String logEntry = bufferedReader.readLine();
                    if (logEntry != null && logEntry.contains("RobotCore")) {
                        // Append filtered log entry with a newline
                        newLogs.append(logEntry).append("\n");
                    }
                }

                // Send new filtered log entries to the dashboard if there are any
                if (newLogs.length() > 0) {
                    sendLogsToDashboard(newLogs.toString());
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
    private void sendLogsToDashboard(String logs) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Log Entries", logs);
        dashboard.sendTelemetryPacket(packet);

        // Optionally, also update the driver station telemetry
        telemetry.addData("Log Entries", logs);
        telemetry.update();
    }
}