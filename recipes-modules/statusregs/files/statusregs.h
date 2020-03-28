#ifndef statusregs_H
#define statusregs_H

#define OFFSET_ID 0x00
#define OFFSET_DISPFREQ 0x01
#define OFFSET_DISPMODE 0x02
#define OFFSET_RXMODE 0x03
#define OFFSET_PHACC1 0x04


#define DEBUG

struct disp_settings {
    bool enabled;
};

struct adc_settings {
    unsigned int clockFreq;
    bool random;
    bool dither;
};

typedef enum {AM = 0} rfMode_t;

struct rf_settings {
    unsigned int ifFreq;
    rfMode_t mode;
    unsigned int transverterOffset;  // Offset added by external transverters
};

static struct statusregs_local {
	unsigned int mem_start;
	unsigned int mem_end;
	void __iomem *base_addr;
    struct disp_settings display;
    struct adc_settings adc;
    struct rf_settings rf;
};

static int status_open(struct inode *inode_p, struct file *file_p);
static int status_release(struct inode *inode_p, struct file *file_p);
static ssize_t status_read(struct file *file_p, char *outBuffer_p, size_t length, loff_t *offset);
static ssize_t status_write(struct file *file_p, const char *inBuffer_p, size_t length, loff_t *offset);
void init_vars(void);
void update_all(void);

#endif //  ifdef statusregs_H