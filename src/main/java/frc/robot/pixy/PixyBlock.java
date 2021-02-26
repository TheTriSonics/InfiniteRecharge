package frc.robot.pixy;

public class PixyBlock {
	public final int type;
	public final int signature;
	public final int x, y;
	public final int width, height;
	public final int angle;
	
	public PixyBlock(int type, int signature, int x, int y, int width, int height, int angle) {
		this.type = type;
		this.signature = signature;
		this.x = x;
		this.y = y;
		this.width = width;
		this.height = height;
		this.angle = angle;
	}

	@Override
	public String toString() {
		return "Block [type=" + type + ", signature=" + signature + ", x=" + x + ", y=" + y + ", width=" + width
				+ ", height=" + height + ", angle=" + angle + "]";
	}
}
