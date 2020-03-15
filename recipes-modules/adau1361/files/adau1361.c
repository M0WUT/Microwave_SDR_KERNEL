/*  adau1361.c - The simplest kernel module.
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation; either version 3 of the License, or
*   (at your option) any later version.

*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License along
*   with this program. If not, see <http://www.gnu.org/licenses/>.

*/
#include "adau1361.h"

/* Standard module information, edit as appropriate */
MODULE_LICENSE("GPL");
MODULE_AUTHOR
    ("Dan McGraw M0WUT");
MODULE_DESCRIPTION
    ("Kernel driver for ADAU1361 audio codec");

#define DRIVER_NAME "adau1361"

// Load default settings
static void adau1361_load_defaults(struct adau1361_local *dev){

	// PLL
	dev->pll.pll_mode = PLL_DISABLED;
	dev->pll.core_clock_ratio = FSx256;

	// Inputs
	//dev->mic_detect_enabled = 0;
	dev->operational_mode = NORMAL;
	dev->left_record_mixer.input_mode = IN_N;
	//dev->left_record_mixer.gain = 0;
	dev->right_record_mixer.input_mode = INPUT_DISABLED;

	// Misc
	dev->bclk_pol = RISING_EDGE;
	dev->lrclk_pol = RISING_EDGE;

	//Outputs
	dev->left_playback_mixer.output_mode = LEFT_DAC;
	//dev->left_playback_mixer.gain = 0;
	dev->right_playback_mixer.output_mode = RIGHT_DAC;
	//dev->right_playback_mixer.gain = 0;

	dev->headphones.enabled = 1;
	dev->headphones.muted = 0;
	dev->headphones.left_volume = 63;
	dev->headphones.right_volume = 63;
	dev->line_out.muted = 1;
}

// Writes all settings in adau1361_local to the device
static int adau1361_update_full(struct adau1361_local *dev_p){

	uint8_t x[6];  // Generic data array
	uint8_t data;  // Generic register for data to send

	////////////////
	// Update PLL //
	////////////////
	
	// Disable core clock
	iic_write_single(&dev_p->iic, ADAU1361_REG_CLOCK_CONTROL, 0);
	
	// Update PLL - must be written to as block of 6 bytes
	

	// Throw error if pll_mode is not PLL_DISABLED
	// I haven't fully implemented the other modes
	// TODO: Add these features
	
	if(dev_p->pll.pll_mode != PLL_DISABLED){
		printk(KERN_ERR "ADAU1361: Only PLL_DISABLED implemented currently");
		return -EINVAL;
	}

	x[0] = (dev_p->pll.pll_denominator >> 8) & 0xFF;
	x[1] = (dev_p->pll.pll_denominator) & 0xFF;
	x[2] = (dev_p->pll.pll_numerator >> 8) & 0xFF;
	x[3] = (dev_p->pll.pll_numerator) & 0xFF;
	x[4] = 	((dev_p->pll.pll_integer << 3) & 0x78) |
			(((dev_p->pll.pll_input_divider - 1) << 1) & 0x6) | 
			(dev_p->pll.pll_mode == PLL_FRACTIONAL ? 1 : 0);
	x[5] = (dev_p->pll.pll_mode != PLL_DISABLED ? 1 : 0);

	iic_write_block(&dev_p->iic, ADAU1361_REG_PLL_CONTROL, x, 6);

	// TODO: Check PLL Lock bit - this is what's preventing PLL being used
	
	// Update Clock Control register
	data = 0;

	// Enable the PLL if needed
	if  (dev_p->pll.pll_mode == PLL_DISABLED) {
	 	data |= 0x80;
	}

	// Set Core Clock Divider ratio
	switch(dev_p->pll.core_clock_ratio) {
		case FSx256:
			data |= 0b00 << 1;
			break;
		case FSx512:
			data |= 0b01 << 1;
			break;
		case FSx768:
			data |= 0b10 << 1;
			break;
		case FSx1024:
			data |= 0x11 << 1;
			break;
	}

	// Enable the core clock
	data |= 0x01;	
	
	iic_write_single(&dev_p->iic, ADAU1361_REG_CLOCK_CONTROL, data);
	// Disable Jack detect - errata says this doesn't work properly
	iic_write_single(&dev_p->iic, ADAU1361_REG_JACK_DETECT, 0x00);

	// Record Power Management
	switch(dev_p->operational_mode) {
		case NORMAL: 
			data = 0;
			break;
		case EXTREME_POWER_SAVING:
			data = 0b01110;
			break;
		case POWER_SAVING:
			data = 0b11110;
			break;
		case ENHANCED_PERFORMANCE:
			data = 0b101000;
			break;
	}
	iic_write_single(&dev_p->iic, ADAU1361_REG_REC_PWR_MGMT, data);

	// Record Mixer Left Control Registers
	// TODO implement gain control
	switch(dev_p->left_record_mixer.input_mode){
		case DIGITAL_MICROPHONE:  // Deliberate fallthrough
		case INPUT_DISABLED:
			x[0] = 0;  // Mute LINP, LINN and disable mixer
			x[1] = 0;  // Mute DIFF and AUX input
			break;
		case IN_P:
			x[0] = 0b01010001;  // 0dB gain for LINP, mute LINN
			x[1] = 0;  // Mute DIFF and AUX input
			break;
		case IN_N:
			x[0] = 0b00001011;  // 0dB gain for LINN, mute LINP
			x[1] = 0;  // Mute DIFF and AUX input
			break;
		case IN_DIFF:
			x[0] = 0b00000001;  // Mute LINP, LINN and enable mixer
			x[1] = 0b00001000;  // 0dB gain for DIFF, mute AUX
			break;
		case IN_AUX:
			x[0] = 0b00000001;  // Mute LINP, LINN and enable mixer
			x[1] = 0b00000101;  // Mute DIFF, 0dB gain for AUX
			break;
	}
	
	iic_write_block(&dev_p->iic, ADAU1361_REG_REC_MIXER_L0, x, 2);

	// Record Mixer Right Control Registers
	// TODO implement gain control
	switch(dev_p->right_record_mixer.input_mode){
		case DIGITAL_MICROPHONE:  // Deliberate fallthrough
		case INPUT_DISABLED:
			x[0] = 0;  // Mute LINP, LINN and disable mixer
			x[1] = 0;  // Mute DIFF and AUX input
			break;
		case IN_P:
			x[0] = 0b01010001;  // 0dB gain for LINP, mute LINN
			x[1] = 0;  // Mute DIFF and AUX input
			break;
		case IN_N:
			x[0] = 0b00001011;  // 0dB gain for LINN, mute LINP
			x[1] = 0;  // Mute DIFF and AUX input
			break;
		case IN_DIFF:
			x[0] = 0b00000001;  // Mute LINP, LINN and enable mixer
			x[1] = 0b00001000;  // 0dB gain for DIFF, mute AUX
			break;
		case IN_AUX:
			x[0] = 0b00000001;  // Mute LINP, LINN and enable mixer
			x[1] = 0b00000101;  // Mute DIFF, 0dB gain for AUX
			break;
	}
	
	iic_write_block(&dev_p->iic, ADAU1361_REG_REC_MIXER_R0, x, 2);

	// Left differential input volume control
	data = (0b010000 << 2);  // 0dB differential gain
	if(dev_p->left_record_mixer.input_mode == IN_DIFF)
		data |= 0b11;  // Enable and unmute differential input
	else
		data |= 0b00; // Disable and mute differential input
	iic_write_single(&dev_p->iic, ADAU1361_REG_LEFT_DIFF_VOL, data);

	// Right differential input volume control
	data = (0b010000 << 2);  // 0dB differential gain
	if(dev_p->right_record_mixer.input_mode == IN_DIFF)
		data |= 0b11;  // Enable and unmute differential input
	else
		data |= 0b00; // Disable and mute differential input
	iic_write_single(&dev_p->iic, ADAU1361_REG_RIGHT_DIFF_VOL, data);

	// Microphone Bias
	if(dev_p->mic_bias_enabled)
		data = 1;
	else
		data = 0;
	if(dev_p->operational_mode == ENHANCED_PERFORMANCE)
		data |= 0b1000;
	iic_write_single(&dev_p->iic, ADAU1361_REG_REC_MIC_BIAS, data);

	// ALC Registers
	// TODO Implement ALC control - for now disable it
	x[0] = 0;  // Disable ALC for both channels, all other settings now irrelevant
	x[1] = 0;
	x[2] = 0;
	x[3] = 0;
	iic_write_block(&dev_p->iic, ADAU1361_REG_ALC0, x, 4);

	// Serial Port Control Registers
	// TODO Implement control of these options rather than hardcoding
	x[0] = 0;
	if(dev_p->bclk_pol == RISING_EDGE)
		x[0] |= 0x10;

	if(dev_p->lrclk_pol == RISING_EDGE)
		x[0] |= 0x80;

	x[1] = (
		(0b001 << 5) |  // 32 clock cycles per frame
		(0 << 4) |  // ADC Left channel first
		(0 << 3) |  // DAC Left channel first
		(0 << 2) |  // MSB first
		(0b01)  // 0 delay between edge of LRCLK and data
	);
	iic_write_block(&dev_p->iic, ADAU1361_REG_SERIAL_PORT0, x, 2);

	// TDM Converter control
	// TODO Implement TDM data support
	x[0] = 0;
	x[1] = 0;
	iic_write_block(&dev_p->iic, ADAU1361_REG_CONV0, x, 2);

	// ADC Control
	// TODO Implement Nice ADC control and support for digital microphones
	data = (
		(0 << 6) |  // Normal ADC polarity
		(1 << 5) |  // Enable ADC 2Hz HPF
		(1 << 4) |  // Normal digital mic polarity
		(0 << 3) | // Don't swap digital microphone channels
		(0 << 2)  // Disable digital microphone
	);

	if(
		dev_p->left_record_mixer.input_mode == DIGITAL_MICROPHONE ||
		dev_p->right_record_mixer.input_mode == DIGITAL_MICROPHONE
	){
		printk(KERN_ERR "ADAU1361: Digital microphone not yet implemented");
		return -EINVAL;
	}

	if(dev_p->left_record_mixer.input_mode != INPUT_DISABLED)
		data |= 0b01;

	if(dev_p->right_record_mixer.input_mode != INPUT_DISABLED)
		data |= 0b10;

	iic_write_single(&dev_p->iic, ADAU1361_REG_ADC_CTL, data);

	// Input digital volume
	// TODO implement input digital volume control
	x[0] = 0;
	x[1] = 0;
	iic_write_block(&dev_p->iic, ADAU1361_REG_LEFT_DIG_VOL, x, 2);

	// Left playback mixer
	switch(dev_p->left_playback_mixer.output_mode) {
		case OUTPUT_DISABLED:
			x[0] = (
				(0 << 6) |  // Mute Right DAC to left mixer
				(0 << 5) |  // Mute Left DAC to left mixer
				(0b0000 << 4) |  // Mute Left AUX to left mixer
				0  // Disable left mixer
			);
			x[1] = (
				(0b0000 << 4) |  // Mute right input mixer to left mixer
				0b0000  // Mute left input mixer to left mixer
			);
			break;

		case LEFT_INPUT_MIXER:
			x[0] = (
				(0 << 6) |  // Mute Right DAC to left mixer
				(0 << 5) |  // Mute Left DAC to left mixer
				(0b0000 << 4) |  // Mute Left AUX to left mixer
				1  // Enable left mixer
			);
			x[1] = (
				(0b0000 << 4) |  // Mute right input mixer to left mixer
				0b0110  // 0dB gain from left input mixer to left mixer
			);
			break;

		case RIGHT_INPUT_MIXER:
			x[0] = (
				(0 << 6) |  // Mute Right DAC to left mixer
				(0 << 5) |  // Mute Left DAC to left mixer
				(0b0000 << 4) |  // Mute Left AUX to left mixer
				1  // Enable left mixer
			);
			x[1] = (
				(0b0110 << 4) |  // 0dB gain from right input mixer to left mixer
				0b0000  // Mute left input mixer to left mixer
			);
			break;

		case AUX:
			x[0] = (
				(0 << 6) |  // Mute Right DAC to left mixer
				(0 << 5) |  // Mute Left DAC to left mixer
				(0b0110 << 4) |  // 0dB gain from Left AUX to left mixer
				1  // Enable left mixer
			);
			x[1] = (
				(0b0000 << 4) |  // Mute left input mixer to left mixer
				0b0000  // Mute left input mixer to left mixer
			);
			break;
			
		case LEFT_DAC:
			x[0] = (
				(0 << 6) |  // Mute Right DAC to left mixer
				(1 << 5) |  // Enable Left DAC to left mixer
				(0b0000 << 4) |  // Mute Left AUX to left mixer
				0  // Disable left mixer
			);
			x[1] = (
				(0b0000 << 4) |  // Mute right input mixer to left mixer
				0b0000  // Mute left input mixer to left mixer
			);
			break;

		case RIGHT_DAC:
			x[0] = (
				(1 << 6) |  // Enable Right DAC to left mixer
				(0 << 5) |  // Mute Left DAC to left mixer
				(0b0000 << 4) |  // Mute Left AUX to left mixer
				0  // Disable left mixer
			);
			x[1] = (
				(0b0000 << 4) |  // Mute right input mixer to left mixer
				0b0000  // Mute left input mixer to left mixer
			);
			break;
	}
	iic_write_block(&dev_p->iic, ADAU1361_REG_PLAY_MIXER_L0, x, 2);

	// Right Playback Mixer
		// Left playback mixer
	switch(dev_p->right_playback_mixer.output_mode) {
		case OUTPUT_DISABLED:
			x[0] = (
				(0 << 6) |  // Mute Right DAC to left mixer
				(0 << 5) |  // Mute Left DAC to left mixer
				(0b0000 << 4) |  // Mute Left AUX to left mixer
				0  // Disable left mixer
			);
			x[1] = (
				(0b0000 << 4) |  // Mute right input mixer to left mixer
				0b0000  // Mute left input mixer to left mixer
			);
			break;

		case LEFT_INPUT_MIXER:
			x[0] = (
				(0 << 6) |  // Mute Right DAC to left mixer
				(0 << 5) |  // Mute Left DAC to left mixer
				(0b0000 << 4) |  // Mute Left AUX to left mixer
				1  // Enable left mixer
			);
			x[1] = (
				(0b0000 << 4) |  // Mute right input mixer to left mixer
				0b0110  // 0dB gain from left input mixer to left mixer
			);
			break;

		case RIGHT_INPUT_MIXER:
			x[0] = (
				(0 << 6) |  // Mute Right DAC to left mixer
				(0 << 5) |  // Mute Left DAC to left mixer
				(0b0000 << 4) |  // Mute Left AUX to left mixer
				1  // Enable left mixer
			);
			x[1] = (
				(0b0110 << 4) |  // 0dB gain from right input mixer to left mixer
				0b0000  // Mute left input mixer to left mixer
			);
			break;

		case AUX:
			x[0] = (
				(0 << 6) |  // Mute Right DAC to left mixer
				(0 << 5) |  // Mute Left DAC to left mixer
				(0b0110 << 4) |  // 0dB gain from Left AUX to left mixer
				1  // Enable left mixer
			);
			x[1] = (
				(0b0000 << 4) |  // Mute left input mixer to left mixer
				0b0000  // Mute left input mixer to left mixer
			);
			break;
			
		case LEFT_DAC:
			x[0] = (
				(0 << 6) |  // Mute Right DAC to left mixer
				(1 << 5) |  // Enable Left DAC to left mixer
				(0b0000 << 4) |  // Mute Left AUX to left mixer
				0  // Disable left mixer
			);
			x[1] = (
				(0b0000 << 4) |  // Mute right input mixer to left mixer
				0b0000  // Mute left input mixer to left mixer
			);
			break;

		case RIGHT_DAC:
			x[0] = (
				(1 << 6) |  // Enable Right DAC to left mixer
				(0 << 5) |  // Mute Left DAC to left mixer
				(0b0000 << 4) |  // Mute Left AUX to left mixer
				0  // Disable left mixer
			);
			x[1] = (
				(0b0000 << 4) |  // Mute right input mixer to left mixer
				0b0000  // Mute left input mixer to left mixer
			);
			break;
	}
	iic_write_block(&dev_p->iic, ADAU1361_REG_PLAY_MIXER_R0, x, 2);

	// L/R Playback Mixer Left
	// TODO allow for cross feed of signals - not sure why this is useful though
	data = (
		(0b00 << 3) |  // Mute right mixer into left line out
	 	(0b10 << 1)  // 0dB loss from left line mixer to left line out
	);
	if(dev_p->line_out.muted == 0)
		data |= 1;  // Enable output mixer

	iic_write_single(&dev_p->iic, ADAU1361_REG_PLAY_LR_MIXER_LEFT, data);

	// L/R Playback Mixer Right
	// TODO allow for cross feed of signals - not sure why this is useful though
	data = (
		(0b10 << 3) |  // 0dB loss from right line mixer to right line out
	 	(0b00 << 1)  // Mute left mixer into right line out
	);
	if(dev_p->line_out.muted == 0)
		data |= 1;  // Enable output mixer

	iic_write_single(&dev_p->iic, ADAU1361_REG_PLAY_LR_MIXER_RIGHT, data);

	// Mono mixer
	if(dev_p->headphones.enabled)
		data = (
			(0b00 << 1) |  // Commmon mode ouput
			1  // Enable mixer
		);
	else
		data = 0;
	iic_write_single(&dev_p->iic, ADAU1361_REG_PLAY_LR_MIXER_MONO, data);

	// Headphone Right
	data = (dev_p->headphones.left_volume << 2) & 0xFC;
	if(dev_p->headphones.muted == 0)
		data |= (1 << 1);

	if(dev_p->headphones.enabled == 1)
		data |= (1 << 0);
	iic_write_single(&dev_p->iic, ADAU1361_REG_HP_LEFT_VOL, data);

	// Headphone Right
	data = (dev_p->headphones.right_volume << 2) & 0xFC;
	if(dev_p->headphones.muted == 0)
		data |= (1 << 1);

	data |= 1;  // Set headphone outputs to headphone mode
	iic_write_single(&dev_p->iic, ADAU1361_REG_HP_RIGHT_VOL, data);

	// Line Out left
	data = (dev_p->line_out.left_volume << 2) & 0xFC;
	if(dev_p->line_out.muted == 0)
		data |= (1 << 1);

	// Yes, I know this does nothing, just acknowledgement that this bit is 0
	data |= 0;  // Set Line Out outputs into line out mode

	iic_write_single(&dev_p->iic, ADAU1361_REG_LINE_LEFT_VOL, data);

	// Line Out Right
	data = (dev_p->line_out.right_volume << 2) & 0xFC;
	if(dev_p->line_out.muted == 0)
		data |= (1 << 1);

	// Yes, I know this does nothing, just acknowledgement that this bit is 0
	data |= 0;  // Set Line Out outputs into line out mode

	iic_write_single(&dev_p->iic, ADAU1361_REG_HP_RIGHT_VOL, data);

	// Mono output
	// TODO: Add support for MONO audio rather than just headphone common mode
	if(dev_p->headphones.enabled)
		data = (
			(1 << 1) |  // Unmute mono output
			1  // Headphone mode
		);
	iic_write_single(&dev_p->iic, ADAU1361_REG_MONO_OUTPUT, data);

	// Playback Pop/Click suppression
	data = (
		(0 << 3) |  // Enable pop suppression
		0b00  // Default slew rate coz I don't know better 
	);
	if(
		dev_p->operational_mode == POWER_SAVING ||
		dev_p->operational_mode == EXTREME_POWER_SAVING
	) {
		data |= (1 << 4);  // Enable low power click suppression
	}
	iic_write_single(&dev_p->iic, ADAU1361_REG_POP_SUPRESS, data);

	// Playback Power Management
	switch(dev_p->operational_mode){
		case NORMAL:
			data = (
				(0b00 << 6) |
				(0b00 << 4) |
				(0b00 << 2)
			);
			break;
		case EXTREME_POWER_SAVING:
			data = (
				(0b01 << 6) |
				(0b01 << 4) |
				(0b11 << 2)
			);
			break;
		case POWER_SAVING:
			data = (
				(0b11 << 6) |
				(0b11 << 4) |
				(0b11 << 2)
			);
			break;
		case ENHANCED_PERFORMANCE:
			data = (
				(0b10 << 6) |
				(0b10 << 4) |
				(0b10 << 2)
			);
			break;
	}
	if(dev_p->right_playback_mixer.output_mode != OUTPUT_DISABLED)
		data |= (1 << 1);
	
	if(dev_p->left_playback_mixer.output_mode != OUTPUT_DISABLED)
		data |= (1 << 0);

	iic_write_single(&dev_p->iic, ADAU1361_REG_PLAY_PWR_MGMT, data);

	// DAC Control
	x[0] = 0;
	if(
		dev_p->left_playback_mixer.output_mode == LEFT_DAC ||
		dev_p->right_playback_mixer.output_mode == LEFT_DAC
	) {
		x[0] |= 0b01;
	}

	if(
		dev_p->left_playback_mixer.output_mode == RIGHT_DAC ||
		dev_p->right_playback_mixer.output_mode == RIGHT_DAC
	) {
		x[0] |= 0b10;
	}
	x[1] = 0;
	x[2] = 0;
	iic_write_block(&dev_p->iic, ADAU1361_REG_DAC0, x, 3);

	// Serial input pin settings
	iic_write_single(&dev_p->iic, ADAU1361_REG_SERIAL_PORT_PAD, 0xAA);  // No pull up/down

	// Control port pin settings
	iic_write_single(&dev_p->iic, ADAU1361_REG_CONTROL_PORT_PAD0, 0xAA);  // No pull up/down
	iic_write_single(&dev_p->iic, ADAU1361_REG_CONTROL_PORT_PAD1, 0x0);  // Low drive strength

	// Jack detect pin settings
	data = (
		(0 << 5) |  // Low drive strength
		(0b10 << 2)  // No pull up-down
	);
	iic_write_single(&dev_p->iic, ADAU1361_REG_JACK_DETECT_PIN, data);

	// Dejitter
	iic_write_single(&dev_p->iic, ADAU1361_REG_DEJITTER, 0x03);  // Don't really know so use defaults

	return 0;
}

// Loads default settings and updates full register set of ADAU1361
static int adau1361_init_device(struct adau1361_local *dev){
	adau1361_load_defaults(dev);
	adau1361_update_full(dev);
	printk(KERN_INFO "ADAU1361: All setup, enabling output now");
	iowrite32(0x01, dev->iic.base_addr + OFFSET_GPO);  // Let FPGA Fabric know it's all good
}

/////////////////////////
// Kernel module stuff //
/////////////////////////

static int adau1361_probe(struct platform_device *pdev)
{
	int r_irq; /* Interrupt resources */
	struct resource *r_mem; /* IO mem resources */
	struct device *dev = &pdev->dev;
	struct device_node *node_p = pdev->dev.of_node;
	struct adau1361_local *adau1361_dev_p = NULL;
	int rc = 0;

	printk(KERN_INFO "Probing ADAU1361 module");
	/* Get iospace for the device */
	r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r_mem) {
		dev_err(dev, "Invalid address passed in reg parameter\n");
		return -ENODEV;
	}
	adau1361_dev_p = (struct adau1361_local *) kmalloc(sizeof(struct adau1361_local), GFP_KERNEL);
	if (!adau1361_dev_p) {
		dev_err(dev, "Cound not allocate adau1361 device\n");
		return -ENOMEM;
	}
	dev_set_drvdata(dev, adau1361_dev_p);
	adau1361_dev_p->iic.mem_start = r_mem->start;
	adau1361_dev_p->iic.mem_end = r_mem->end;

	if (!request_mem_region(adau1361_dev_p->iic.mem_start,
				adau1361_dev_p->iic.mem_end - adau1361_dev_p->iic.mem_start + 1,
				DRIVER_NAME)) {
		dev_err(dev, "Couldn't lock memory region at %p\n",
			(void *)adau1361_dev_p->iic.mem_start);
		rc = -EBUSY;
		goto error1;
	}

	adau1361_dev_p->iic.base_addr = ioremap(adau1361_dev_p->iic.mem_start, adau1361_dev_p->iic.mem_end - adau1361_dev_p->iic.mem_start + 1);
	if (!adau1361_dev_p->iic.base_addr) {
		dev_err(dev, "adau1361: Could not allocate iomem\n");
		rc = -EIO;
		goto error2;
	}

	// Request Mutex for the I2C TX Lock
	mutex_init(&adau1361_dev_p->iic.tx_lock);

	/* Get IRQ for the device */
	//r_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	r_irq = platform_get_irq(pdev, 0);
	if (r_irq < 0) {
		dev_info(dev, "no IRQ found\n");
		dev_info(dev, "skeleton at 0x%08x mapped to 0x%08x\n",
			(unsigned int __force)adau1361_dev_p->iic.mem_start,
			(unsigned int __force)adau1361_dev_p->iic.base_addr);
		return 0;
	}

	adau1361_dev_p->iic.irq = r_irq;
	// rc = request_irq(adau1361_dev_p->iic.irq, &iic_irq, 0, DRIVER_NAME, &adau1361_dev_p->iic);
	rc = devm_request_threaded_irq(&pdev->dev, r_irq, iic_irq, iic_irq_process, IRQF_ONESHOT, DRIVER_NAME, &adau1361_dev_p->iic);
	if (rc) {
		dev_err(dev, "testmodule: Could not allocate interrupt %d.\n",
			adau1361_dev_p->iic.irq);
		goto error3;
	}

	// Read device tree parameters and sanitise
	rc = of_property_read_u32(node_p, "slave_address", &adau1361_dev_p->iic.slave_address);
	if (rc) {
		dev_err(&pdev->dev, "Can't parse I2C slave address\n");
	}
	#ifdef DEBUG
		printk(KERN_INFO "ADAU1361: slave I2C address loaded as %04X", adau1361_dev_p->iic.slave_address);
	#endif

	// Initialise I2C peripheral
	iic_init(&adau1361_dev_p->iic);
	adau1361_init_device(adau1361_dev_p);

	printk(KERN_INFO "ADAU1361: Loaded successfully");

	return 0;


error3:
	free_irq(adau1361_dev_p->iic.irq, adau1361_dev_p);
error2:
	release_mem_region(adau1361_dev_p->iic.mem_start, adau1361_dev_p->iic.mem_end - adau1361_dev_p->iic.mem_start + 1);
error1:
	kfree(adau1361_dev_p);
	dev_set_drvdata(dev, NULL);
	printk(KERN_ERR "ADAU1361: Failed to load");
	return rc;
}

static int adau1361_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adau1361_local *adau1361_dev_p = dev_get_drvdata(dev);
	iounmap(adau1361_dev_p->iic.base_addr);
	release_mem_region(adau1361_dev_p->iic.mem_start, adau1361_dev_p->iic.mem_end - adau1361_dev_p->iic.mem_start + 1);
	kfree(adau1361_dev_p);
	dev_set_drvdata(dev, NULL);
	return 0;
}


static struct of_device_id adau1361_of_match[] = {
	{ .compatible = "m0wut,adau1361", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, adau1361_of_match);

static struct platform_driver adau1361_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table	= adau1361_of_match,
	},
	.probe		= adau1361_probe,
	.remove		= adau1361_remove,
};

static int __init adau1361_init(void)
{
	return platform_driver_register(&adau1361_driver);
}


static void __exit adau1361_exit(void)
{
	platform_driver_unregister(&adau1361_driver);
	printk(KERN_ALERT "Goodbye module world.\n");
}

module_init(adau1361_init);
module_exit(adau1361_exit);
