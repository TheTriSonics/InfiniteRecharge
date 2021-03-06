// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.stream.Collectors;
import frc.robot.pixy.PixyBlock;
import frc.robot.pixy.PixyLibrary;
import com.sun.jna.Native;
import com.sun.jna.ptr.ByteByReference;
import com.sun.jna.ptr.IntByReference;
import com.sun.jna.ptr.ShortByReference;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import java.util.List;
import java.util.Arrays;

/** Add your docs here. */
public class Pixycam extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private int uid;
  public static final PixyLibrary PIXY_NATIVE = (PixyLibrary) Native.loadLibrary("pixyusb", PixyLibrary.class);
  private final ByteBuffer blocksBuffer = ByteBuffer.allocateDirect(MAX_BLOCKS * BLOCK_STRUCT_SIZE).order(ByteOrder.nativeOrder());
  private static final int MAX_PIXIES = 10;
	private static final int MAX_ENUMERATE_RETRIES = 10;
	private static final int FRAME_WIDTH = 320, FRAME_HEIGHT = 200;
	private static final int FRAME_RATE_PERIOD = 100;
	private static final int FRAME_MAX_RETRIES = 2, COMMAND_MAX_RETRIES = 10;
	private static final short MAX_BLOCKS = 200;
  private static final int BLOCK_STRUCT_SIZE = 14;

  private boolean initialized = false;

  public Pixycam(int uid) {
    this.uid = uid;
  }

  public void init() {
    for (int c = 0; c < 2; c++) {
      System.out.println(Integer.toHexString(uid));
      int ret = PIXY_NATIVE.pixy_command(uid, "run", 0, new IntByReference(), 0);
      if (ret == 0) {
        initialized = true;
        break;
      } else {
        System.out.println("Retry pixy init.");
      }
    }
  }
  
  public List<PixyBlock> getBlocks() {
    if (!initialized) {
      this.init();
    }
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
  
  public static int[] enumerate() {
		int[] uids = new int[MAX_PIXIES];
		int pixyCount = PIXY_NATIVE.pixy_enumerate(uids.length, uids);
		String uidsString = Arrays.stream(uids).limit(pixyCount).mapToObj(x -> String.format("0x%08X", x)).collect(Collectors.joining(", "));
		System.out.printf("pixy_enumerate: count=%d, uids=[%s]\n", pixyCount, uidsString);
		int[] pixyUIDs = new int[pixyCount];
		System.arraycopy(uids, 0, pixyUIDs, 0, pixyCount);
		return pixyUIDs;
  }
}
