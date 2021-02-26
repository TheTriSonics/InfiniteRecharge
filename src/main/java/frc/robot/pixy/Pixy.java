package frc.robot.pixy;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.concurrent.Executors;
import java.util.concurrent.LinkedTransferQueue;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TransferQueue;
import java.util.function.IntSupplier;
import java.util.stream.Collectors;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.sun.jna.Native;
import com.sun.jna.ptr.ByteByReference;
import com.sun.jna.ptr.IntByReference;
import com.sun.jna.ptr.ShortByReference;

import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;

public class Pixy {
	private static final int MAX_PIXIES = 10;
	private static final int MAX_ENUMERATE_RETRIES = 10;
	private static final int FRAME_WIDTH = 320, FRAME_HEIGHT = 200;
	private static final int FRAME_RATE_PERIOD = 100;
	private static final int FRAME_MAX_RETRIES = 2, COMMAND_MAX_RETRIES = 10;
	private static final short MAX_BLOCKS = 200;
	private static final int BLOCK_STRUCT_SIZE = 14;
	
	private static final Scalar[] BLOCK_COLORS = {
			new Scalar(0xFF, 0xFF, 0xFF),
			new Scalar(0x00, 0x00, 0xFF),
			new Scalar(0x00, 0x80, 0xFF),
			new Scalar(0x00, 0xFF, 0xFF),
			new Scalar(0x00, 0xFF, 0x00),
			new Scalar(0xFF, 0xFF, 0x00),
			new Scalar(0xFF, 0x00, 0x00),
			new Scalar(0xFF, 0x00, 0xFF),
	};
	
	private static final PixyLibrary PIXY_NATIVE = (PixyLibrary) Native.loadLibrary("pixyusb", PixyLibrary.class);
	
	static {
		Runtime.getRuntime().addShutdownHook(new Thread(new Runnable() {
			@Override
			public void run() {
				PIXY_NATIVE.pixy_close();
			}
		}));
	}
	
	public final int uid;
	
	private final byte[] pixels = new byte[FRAME_WIDTH * FRAME_HEIGHT];
	private final Mat rawFrame = new Mat(FRAME_HEIGHT, FRAME_WIDTH, CvType.CV_8UC1);
	private final Mat colorFrame = new Mat(FRAME_HEIGHT, FRAME_WIDTH, CvType.CV_8UC3);
	private final CvSource frameSource;
	
	private final ByteBuffer blocksBuffer = ByteBuffer.allocateDirect(MAX_BLOCKS * BLOCK_STRUCT_SIZE).order(ByteOrder.nativeOrder());
	
	private final ScheduledExecutorService executor = Executors.newScheduledThreadPool(2, new ThreadFactory() {
		@Override
		public Thread newThread(Runnable r) {
			Thread t = Executors.defaultThreadFactory().newThread(r);
			t.setDaemon(true);
			return t;
		}
	});
	
	private final Runnable frameGrabberTask = new Runnable() {
		@Override
		public void run() {
			if (isFrameGrabberRunning) {
				int ret = PIXY_NATIVE.pixy_cam_update_frame(uid);
				if (ret == 0) {
					frameErrorCount = 0;
				} else {
					frameErrorCount++;
					if (frameErrorCount >= FRAME_MAX_RETRIES) {
						PIXY_NATIVE.pixy_cam_reset_frame_wait(uid);
					}
				}
				
				PIXY_NATIVE.pixy_cam_get_frame(uid, pixels);
				
				rawFrame.put(0, 0, pixels);
				Imgproc.cvtColor(rawFrame, colorFrame, Imgproc.COLOR_BayerBG2RGB);
				for (PixyBlock block : getBlocks()) {
					if (block.signature >= 0 && block.signature < BLOCK_COLORS.length) {
						Imgproc.rectangle(
								colorFrame,
								new Point(block.x - block.width / 2, block.y - block.height / 2),
								new Point(block.x + block.width / 2, block.y + block.height / 2),
								BLOCK_COLORS[block.signature],
								2
						);
					}
				}
				
				frameSource.putFrame(colorFrame);
			}
		}
	};
	private volatile boolean isFrameGrabberRunning = false;
	private int frameErrorCount = 0;
	
	private final Runnable commandTask = new Runnable() {
		@Override
		public void run() {
			while (true) {
				try {
					IntSupplier command = commandQueue.take();
					for (int i = 0; i < COMMAND_MAX_RETRIES; i++) {
						int ret = command.getAsInt();
						if (ret == 0) {
							break;
						}
					}
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		}
	};
	private final TransferQueue<IntSupplier> commandQueue = new LinkedTransferQueue<>();
	
	public static int[] enumerate() {
		int[] uids = new int[MAX_PIXIES];
		int pixyCount = PIXY_NATIVE.pixy_enumerate(uids.length, uids);
		String uidsString = Arrays.stream(uids).limit(pixyCount).mapToObj(x -> String.format("0x%08X", x)).collect(Collectors.joining(", "));
		System.out.printf("pixy_enumerate: count=%d, uids=[%s]\n", pixyCount, uidsString);
		int[] pixyUIDs = new int[pixyCount];
		System.arraycopy(uids, 0, pixyUIDs, 0, pixyCount);
		return pixyUIDs;
	}
	
	public static boolean ensureAvailable(int... uids) {
		List<Integer> required = Arrays.stream(uids).boxed().collect(Collectors.toList());
		for (int i = 0; i < MAX_ENUMERATE_RETRIES; i++) {
			Set<Integer> found = Arrays.stream(enumerate()).boxed().collect(Collectors.toSet());
			if (found.containsAll(required)) {
				return true;
			}
		}
		return false;
	}
	
	public Pixy(int uid) {
		this.uid = uid;
		frameSource = CameraServer.getInstance().putVideo("Pixy_" + String.format("%08X", uid), FRAME_WIDTH, FRAME_HEIGHT);
		executor.scheduleAtFixedRate(frameGrabberTask, 0, FRAME_RATE_PERIOD, TimeUnit.MILLISECONDS);
		executor.submit(commandTask);
	}
	
	public void clearCommandQueue() {
		commandQueue.clear();
	}
	
	public void startFrameGrabber() {
		PIXY_NATIVE.pixy_cam_reset_frame_wait(uid);
		isFrameGrabberRunning = true;
	}
	
	public void stopFrameGrabber() {
		isFrameGrabberRunning = false;
	}
	
	public void startBlockProgram() {
		commandQueue.add(() -> PIXY_NATIVE.pixy_command(uid, "run", 0, new IntByReference(), 0));
	}
	
	public void stopBlockProgram() {
		commandQueue.add(() -> PIXY_NATIVE.pixy_command(uid, "stop", 0, new IntByReference(), 0));
	}
	
	public List<PixyBlock> getBlocks() {
		int blockCount = PIXY_NATIVE.pixy_get_blocks(uid, MAX_BLOCKS, blocksBuffer);
		PixyBlock[] returnBlocks = new PixyBlock[Math.max(blockCount, 0)];
		for (int i = 0; i < blockCount; i++) {
			PixyBlock block = new PixyBlock(
				blocksBuffer.getShort() & 0xFFFF, blocksBuffer.getShort() & 0xFFFF,
				blocksBuffer.getShort() & 0xFFFF, blocksBuffer.getShort() & 0xFFFF,
				blocksBuffer.getShort() & 0xFFFF, blocksBuffer.getShort() & 0xFFFF,
				blocksBuffer.getShort()
			);
			returnBlocks[i] = block;
		}
		blocksBuffer.rewind();
		return Arrays.asList(returnBlocks);
	}
	
	public boolean getAutoWhiteBalance() {
		return PIXY_NATIVE.pixy_cam_get_auto_white_balance(uid) == 0 ? false : true;
	}
	public void setAutoWhiteBalance(boolean enabled) {
		commandQueue.add(() -> PIXY_NATIVE.pixy_cam_set_auto_white_balance(uid, enabled ? (byte) 1 : (byte) 0));
	}
	
	public WhiteBalanceSetting getWhiteBalanceValue() {
		int value = PIXY_NATIVE.pixy_cam_get_white_balance_value(uid);
		return new WhiteBalanceSetting((value & 0x00FF00) << 8, (value & 0x0000FF) << 8, (value & 0xFF0000) >> 16);
	}
	public void setWhiteBalanceValue(WhiteBalanceSetting value) {
		commandQueue.add(() -> PIXY_NATIVE.pixy_cam_set_white_balance_value(uid, (byte) value.red, (byte) value.green, (byte) value.blue));
	}
	
	public boolean getAutoExposure() {
		return PIXY_NATIVE.pixy_cam_get_auto_exposure_compensation(uid) == 0 ? false : true;
	}
	public void setAutoExposure(boolean enabled) {
		commandQueue.add(() -> PIXY_NATIVE.pixy_cam_set_auto_exposure_compensation(uid, enabled ? (byte) 1 : (byte) 0));
	}
	
	public ExposureSetting getExposureCompensation() {
		ByteByReference gain = new ByteByReference();
		ShortByReference compensation = new ShortByReference();
		PIXY_NATIVE.pixy_cam_get_exposure_compensation(uid, gain, compensation);
		return new ExposureSetting(gain.getValue() & 0xFF, compensation.getValue() & 0xFFFF);
	}
	public void setExposureCompensation(ExposureSetting value) {
		commandQueue.add(() -> PIXY_NATIVE.pixy_cam_set_exposure_compensation(uid, (byte) value.gain, (short) value.compensation));
	}
	
	public void setLEDColor(int red, int green, int blue) {
		commandQueue.add(() -> PIXY_NATIVE.pixy_led_set_RGB(uid, (byte) red, (byte) green, (byte) blue));
	}
	
	public void setLEDBrightness(int current) {
		commandQueue.add(() -> PIXY_NATIVE.pixy_led_set_max_current(uid, current));
	}
	
	public int getServoPosition(int channel) {
		return PIXY_NATIVE.pixy_rcs_get_position(uid, (byte) channel);
	}
	public void setServoPosition(int channel, int position) {
		commandQueue.add(() -> PIXY_NATIVE.pixy_rcs_set_position(uid, (byte) channel, (short) position));
	}
	
	public void setServoFrequency(int frequency) {
		commandQueue.add(() -> PIXY_NATIVE.pixy_rcs_set_frequency(uid, (short) frequency));
	}
	
	public static class WhiteBalanceSetting {
		public final int red, green, blue;

		public WhiteBalanceSetting(int red, int green, int blue) {
			this.red = red;
			this.green = green;
			this.blue = blue;
		}
	}
	
	public static class ExposureSetting {
		public final int gain, compensation;

		public ExposureSetting(int gain, int compensation) {
			this.gain = gain;
			this.compensation = compensation;
		}
	}
}
