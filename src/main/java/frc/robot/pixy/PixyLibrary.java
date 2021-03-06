package frc.robot.pixy;

import java.nio.Buffer;

import com.sun.jna.Library;
import com.sun.jna.ptr.ByteByReference;
import com.sun.jna.ptr.ShortByReference;

public interface PixyLibrary extends Library {
	int pixy_enumerate(int max_pixy_count, int[] uids);
	void pixy_close();
	int pixy_blocks_are_new(int uid);
	int pixy_get_blocks(int uid, short max_blocks, Buffer blocks);
	int pixy_cam_update_frame(int uid);
	int pixy_cam_get_frame(int uid, byte[] frame);
	int pixy_cam_reset_frame_wait(int uid);
	int pixy_command(int uid, String name, Object... args);
	void pixy_error(int error_code);
	int pixy_led_set_RGB(int uid, byte red, byte green, byte blue);
	int pixy_led_set_max_current(int uid, int current);
	int pixy_led_get_max_current(int uid);
	int pixy_cam_set_auto_white_balance(int uid, byte value);
	int pixy_cam_get_auto_white_balance(int uid);
	int pixy_cam_get_white_balance_value(int uid);
	int pixy_cam_set_white_balance_value(int uid, byte red, byte green, byte blue);
	int pixy_cam_set_auto_exposure_compensation(int uid, byte enable);
	int pixy_cam_get_auto_exposure_compensation(int uid);
	int pixy_cam_set_exposure_compensation(int uid, byte gain, short compensation);
	int pixy_cam_get_exposure_compensation(int uid, ByteByReference gain, ShortByReference compensation);
	int pixy_cam_set_brightness(int uid, byte brightness);
	int pixy_cam_get_brightness(int uid);
	int pixy_rcs_get_position(int uid, byte channel);
	int pixy_rcs_set_position(int uid, byte channel, short position);
	int pixy_rcs_set_frequency(int uid, short frequency);
	int pixy_get_firmware_version(int uid, ShortByReference major, ShortByReference minor, ShortByReference build);
}
