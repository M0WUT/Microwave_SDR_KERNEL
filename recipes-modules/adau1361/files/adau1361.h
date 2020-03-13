#ifndef ADAU1361_H
#define ADAU1361_H

#include "axi_iic.h"

typedef enum {NORMAL, EXTREME_POWER_SAVING, POWER_SAVING, ENHANCED_PERFORMANCE} op_mode_t;
typedef enum {INPUT_DISABLED, IN_P, IN_N, IN_DIFF, IN_AUX, DIGITAL_MICROPHONE} adc_input_t; 
typedef enum {FALLING_EDGE, RISING_EDGE} clk_polarity_t;
typedef enum {OUTPUT_DISABLED, LEFT_INPUT_MIXER, RIGHT_INPUT_MIXER, AUX, LEFT_DAC, RIGHT_DAC} output_t;
typedef enum {PLL_DISABLED, PLL_INTEGER, PLL_FRACTIONAL} pll_mode_t;
typedef enum {FSx256, FSx512, FSx768, FSx1024} core_clock_ratio_t;


struct pll_settings {
    // Let X = pll_input_divider
    // Let R = pll_integer
    // Let N = pll_nominator
    // Let M = pll_denominator
    // Let f = input clock frequency 

    // pll_mode = PLL_DISABLED, core clock = f
    // pll_mode = PLL_INTEGER, core clock = f*R/X
    // pll_mode = PLL_FRACTIONAL, core clock = f * (R+N/M) / X

    pll_mode_t pll_mode;
    int pll_integer;
    uint16_t pll_denominator;
    uint16_t pll_nominator;
    uint8_t pll_input_divider;

    core_clock_ratio_t core_clock_ratio;
};

struct input_mixer {
    adc_input_t input_mode;  // Which source goes to the mixer input
    int gain;  // Gain (in dB) between input and mixer
};

struct output_mixer {
    output_t output_mode;
    int gain;
};

struct headphone_settings {
    bool enabled;  // 1 = output enabled
    bool muted;  // 1 = outputs muted
    uint8_t left_volume;  // 0 = min volume, 63 = max volume
    uint8_t right_volume;  // 0 = min volume, 63 = max volume
};

struct line_out_settings {
    bool muted;  // 1 = outputs muted
    uint8_t left_volume;  // 0 = min volume, 63 = max volume
    uint8_t right_volume;  // 0 = min volume, 63 = max volume
};

struct adau1361_local {

    // I2C settings
	struct iic_local iic;

    // General settings
    struct pll_settings pll;
    bool mic_detect_enabled;
    op_mode_t operational_mode;

    // Input stuff
    struct input_mixer left_record_mixer;
    struct input_mixer right_record_mixer;
    bool mic_bias_enabled;
    clk_polarity_t bclk_pol;  // Which edge data is clocked in on
    clk_polarity_t lrclk_pol;  // Which edge is start of left channel data

    // Output stuff
    struct output_mixer left_playback_mixer;
    struct output_mixer right_playback_mixer;
    struct headphone_settings headphones;
    struct line_out_settings line_out;
};

static void adau1361_load_defaults(struct adau1361_local *dev);

#endif  // #ifdef ADAU1361_H