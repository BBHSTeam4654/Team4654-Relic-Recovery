package org.firstinspires.ftc.teamcode;

import android.graphics.Camera;
import android.util.Log;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.CryptoboxDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Arrays;

@Autonomous(name="DogeCV Test", group="Doge")
public class DogeTest extends OpMode {

	private CryptoboxDetector cd;

	@Override
	public void init() {
		cd = new CryptoboxDetector();
		cd.rotateMat = true;
		cd.detectionMode = CryptoboxDetector.CryptoboxDetectionMode.BLUE;
		cd.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
	}

	@Override
	public void start() {
		cd.enable();
	}

	@Override
	public void stop() {
		Log.d("DogeTest", "Stopping...");
		cd.disable();
		try {
			Files.write(Paths.get("/sdcard/FIRST/dogeOutput.txt"), cd.getOutput().getBytes());
		} catch (IOException e) {}
	}
}
