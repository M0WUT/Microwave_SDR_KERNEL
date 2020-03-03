#ifndef ADAU1361H
#define ADAU1361H

/////////////////////////////////////////
// Registers offsets from base address //
/////////////////////////////////////////

#define OFFSET_GIE 0x01C
#define OFFSET_ISR 0x020
#define OFFSET_IER 0x028
#define OFFSET_SOFTR 0x040
#define OFFSET_CR 0x100
#define OFFSET_SR 0x104
#define OFFSET_TX_FIFO 0x108
#define OFFSET_RX_FIFO 0x10C
#define OFFSET_ADR 0x110
#define OFFSET_TX_FIFO_OCY 0x114
#define OFFSET_RX_FIFO_OCY 0x118
#define OFFSET_TEN_ADR 0x11C
#define OFFSET_RX_FIFO_PIRQ 0x120
#define OFFSET_GPO 0x124
#define OFFSET_TSUSTA 0x128
#define OFFSET_TSUSTO 0x12C
#define OFFSET_THDSTA 0x130
#define OFFSET_TSUDAT 0x134
#define OFFSET_TBUF 0x138
#define OFFSET_THIGH 0x13C
#define OFFSET_TLOW 0x140
#define OFFSET_THDDAT 0x144

/////////////////////////
// Function prototypes //
/////////////////////////

// Basic read/write to memory location
static void reg_write(struct adau1361_local* dev, uint32_t offset, uint32_t data);
static uint32_t reg_read(struct adau1361_local* dev, uint32_t offset);





#endif  // define ADAU1361H