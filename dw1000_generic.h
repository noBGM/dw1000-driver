#ifndef __DW1000_GENERIC_H__
#define __DW1000_GENERIC_H__

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/of_device.h>
#include <linux/gpio/consumer.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/if_arp.h>
#include <linux/skbuff.h>
#include <linux/uaccess.h>
#include <linux/kfifo.h>
#include <linux/if_ether.h>
#include <linux/atomic.h>
#include <linux/wait.h> 
#include <linux/dmaengine.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/highmem.h>
#include <linux/capability.h>    

// --- 通用常量定义 ---
#define DRIVER_NAME "dw1000-generic-hybrid"
#define DW1000_MAX_FRAME_LEN  (1023 + 2)  // Maximum IEEE 802.15.4 UWB frame length + CRC
#define DW1000_DEFAULT_MTU    127   // Default MTU size
#define DW1000_HYBRID_HDR_LEN  sizeof(struct dw1000_hybrid_hdr)   // Hybrid frame header length (updated to match struct)
#define MAX_PENDING_TX        8     // Maximum pending send requests
#define RING_BUFFER_SIZE      (1024*1024) // 1MB ring buffer
#define NAPI_WEIGHT           16    // NAPI polling weight
#define NET_IP_ALIGN          2     // IP header alignment

// --- Driver Configuration Constants ---
#define DW1000_DEFAULT_SPI_HZ       8000000
#define DW1000_DEFAULT_PRF          1        // 0=16MHz, 1=64MHz
#define DW1000_DEFAULT_PREAMBLE_LEN 256U     // Default Preamble Symbols
#define DW1000_DEFAULT_PREAMBLE_CODE 4       // Default preamble code index
#define DW1000_DEFAULT_DATA_RATE    0        // 0=110k, 1=850k, 2=6.8M
#define DW1000_DEFAULT_TX_POWER_REG 0x0E     // Default TX Power register value
#define DW1000_SMART_TX_POWER_REG   0x1F     // TX Power register value for smart power
#define DW1000_DEFAULT_PAN_ID       0xDECA
#define DW1000_DEFAULT_SFD_TIMEOUT  64       // Default SFD timeout
#define DW1000_DEFAULT_TX_DMA_BUF_SIZE 2048
#define DW1000_DEFAULT_RX_DMA_BUF_SIZE 4096
#define DW1000_TX_TEST_FRAME_SIZE   64
#define DW1000_TX_TEST_DELAY_MS     10
#define DW1000_RX_TEST_POLL_MS      100
#define DW1000_MAX_RX_TEST_MS       60000
#define DW1000_MAX_TX_TEST_COUNT    1000

// --- Buffer Sizes ---
#define DW1000_RX_BUFFER_SIZE     (4096)
#define DW1000_TX_BUFFER_SIZE     (4096)
#define DW1000_RX_RING_SIZE       (1024 * 1024)

// --- DW1000 registers and commands ---
// Register ID
#define RG_DEV_ID       0x00  // Device ID register
#define RG_SYS_CFG      0x04  // System configuration register
#define RG_SYS_CTRL     0x0D  // System control register
#define RG_SYS_STATUS   0x0F  // System status register
#define RG_SYS_MASK     0x0E  // Interrupt mask register
#define RG_TX_FCTRL     0x08  // Transmit frame control register
#define RG_TX_BUFFER    0x09  // Transmit data buffer
#define RG_RX_FINFO     0x10  // Receive frame information register
#define RG_RX_BUFFER    0x11  // Receive data buffer
#define RG_TX_POWER     0x1E  // Transmit power control register
#define RG_CHAN_CTRL    0x1F  // Channel control register
#define RG_PANADR       0x03  // PAN address register
#define RG_PMSC         0x36  // Power management system control register

// --- Register Bits and Masks (Moved from .c) ---
// SYS_CFG Register (0x04)
#define SYS_CFG_RXDBUFFEN      (1 << 0)  // RX Double Buffer Enable (Bit 0)
#define SYS_CFG_FFEN           (1 << 5)  // Frame Filtering Enable (Bit 5)
#define SYS_CFG_FFBC           (1 << 6)  // Frame Filtering Beacon Enable (Bit 6)
#define SYS_CFG_FFAB           (1 << 7)  // Frame Filtering Data Enable (Bit 7)
#define SYS_CFG_FFAA           (1 << 8)  // Frame Filtering ACK Enable (Bit 8)
#define SYS_CFG_FFAM           (1 << 9)  // Frame Filtering MAC Enable (Bit 9)
#define SYS_CFG_PHR_MODE_SHIFT 12
#define SYS_CFG_PHR_MODE_MASK  (0x3 << SYS_CFG_PHR_MODE_SHIFT)
#define SYS_CFG_PHR_MODE_STD   (0x0 << SYS_CFG_PHR_MODE_SHIFT)
#define SYS_CFG_PHR_MODE_EXT   (0x3 << SYS_CFG_PHR_MODE_SHIFT)

// CHAN_CTRL Register (0x1F)
#define CHAN_CTRL_CH_NUM_MASK    0x0F      // Channel Number Mask (Bits 0-3)
#define CHAN_CTRL_PRF_SHIFT      4
#define CHAN_CTRL_PRF_MASK       (0x03 << CHAN_CTRL_PRF_SHIFT) // PRF Mask (Bits 4-5)
#define CHAN_CTRL_RX_PRCODE_MASK 0x1F      // RX Preamble Code Mask (Bits 8-12)
#define CHAN_CTRL_TX_PRCODE_SHIFT 13
#define CHAN_CTRL_TX_PRCODE_MASK (0x1F << CHAN_CTRL_TX_PRCODE_SHIFT) // TX Preamble Code Mask (Bits 13-17)

// TX_FCTRL Register (0x08 - 5 bytes)
#define TX_FCTRL_TXFLEN_MASK   0x03FF    // TX Frame Length (Bits 0-9)
#define TX_FCTRL_TXBR_SHIFT    5         // TX Data Rate Shift (Byte 1, Bits 5-6)
#define TX_FCTRL_TXBR_MASK     (0x03 << TX_FCTRL_TXBR_SHIFT)
#define TX_FCTRL_TR_BIT        (1 << 7)  // TX Ranging Bit (Byte 1, Bit 7)
#define TX_FCTRL_PE_SHIFT      4         // Preamble Length Shift (Byte 2, Bits 4-5)
#define TX_FCTRL_PE_MASK       (0x03 << TX_FCTRL_PE_SHIFT)
#define TX_FCTRL_PE_64         0x00      // PE code for 64 symbols
#define TX_FCTRL_PE_128        0x01      // PE code for 128 symbols
#define TX_FCTRL_PE_256        0x02      // PE code for 256 symbols
#define TX_FCTRL_PE_512        0x03      // PE code for 512 symbols
#define TX_FCTRL_PE_NS_00      0x00      // PE code for non-standard (e.g., 1024, 4096)
#define TX_FCTRL_PE_NS_01      0x01      // PE code for non-standard (e.g., 1536)
#define TX_FCTRL_PE_NS_10      0x02      // PE code for non-standard (e.g., 2048)
#define TX_FCTRL_TXPRF_SHIFT   2         // TX PRF Shift (Byte 2, Bits 2-3)
#define TX_FCTRL_TXPRF_MASK    (0x03 << TX_FCTRL_TXPRF_SHIFT)
#define TX_FCTRL_TXPSR_SHIFT   6         // TX Preamble Symbol Repetitions Shift (Byte 2, Bits 6-7) - Corrected Shift?
#define TX_FCTRL_TXPSR_MASK    (0x03 << TX_FCTRL_TXPSR_SHIFT)
#define TX_FCTRL_TXPSR_STD     0x01      // TXPSR for standard lengths (e.g., 64-512)
#define TX_FCTRL_TXPSR_1024    0x02      // TXPSR for 1024, 1536, 2048 lengths
#define TX_FCTRL_TXPSR_4096    0x03      // TXPSR for 4096 length
#define TX_FCTRL_TXPSR_N_16MHZ 0x01      // TXPSR value for 16MHz PRF with std preamble
#define TX_FCTRL_TXPSR_N_64MHZ 0x01      // TXPSR value for 64MHz PRF with std preamble (usually same for std)
#define RG_TX_FCTRL_LEN        5         // Length of TX_FCTRL register

// TX_POWER Register (0x1E) Bit Shifts
#define TX_POWER_PA_SHIFT       8     // Power Amplifier shift
#define TX_POWER_MIXER_SHIFT    16    // Mixer Power shift  
#define TX_POWER_DA_SHIFT       24    // Digital Amplifier shift
// RX_FINFO Register (0x10) Masks
#define RX_FINFO_RXFLEN_MASK    0x03FF    // RX Frame Length Mask (Bits 0-9)
// SYS_CTRL Register (0x0D) - Updated naming
#define SYS_CTRL_TXSTRT         (1 << 1)  // TX Start
#define SYS_CTRL_TRXOFF         (1 << 6)  // Force Transceiver Off  
#define SYS_CTRL_RXENAB         (1 << 8)  // Enable Receiver
// Keep old names for compatibility
#define SYS_CTRL_TXSTRT_BIT     SYS_CTRL_TXSTRT
#define SYS_CTRL_TRXOFF_BIT     SYS_CTRL_TRXOFF
#define SYS_CTRL_RXENAB_BIT     SYS_CTRL_RXENAB
// SPI Command Format
#define SPI_CMD_WRITE          0x80      // Write command prefix (bit 7 = 1)
#define SPI_CMD_READ           0x00      // Read command prefix (bit 7 = 0)
#define SPI_CMD_RW_BIT         0x80
#define SPI_CMD_REG_MASK       0x3F      // Register address mask (6 bits, 0-5)
#define SPI_CMD_SUB_EXT_BIT    0x40      // Extended Sub-Address Bit (bit 6)
#define SPI_CMD_SUB_SHORT_MASK 0x7F      // Short Sub-Address Mask (7 bits, 0-6 of byte 1)
#define SPI_CMD_SUB_LONG_MASK  0x7FFF    // Long Sub-Address Mask (15 bits, 0-6 of byte 1, 0-7 of byte 2)
#define SPI_CMD_SUB_EXT1_BIT   0x80      // Bit 7 of byte 1 indicates 2nd sub-address byte follows
#define SPI_CMD_SUB_EXT2_BIT   0x80      // Bit 7 of byte 2 indicates 3rd sub-address byte follows (Not standard DW1000?)
// SPI Command aliases for compatibility
#define SPI_CMD_SUB_SHORT_EXT_BIT   SPI_CMD_SUB_EXT1_BIT
// Frame Control Field (IEEE 802.15.4 - simplified)
#define FRAME_CTRL_TYPE_BEACON (0x0000)
#define FRAME_CTRL_TYPE_DATA   (0x0001)
#define FRAME_CTRL_TYPE_ACK    (0x0002)
#define FRAME_CTRL_TYPE_MAC_CMD (0x0003)
#define FRAME_CTRL_TYPE_MASK   (0x0007)
#define FRAME_CTRL_SEC_EN      (1 << 3)
#define FRAME_CTRL_FRAME_PEND  (1 << 4)
#define FRAME_CTRL_ACK_REQ     (1 << 5)
#define FRAME_CTRL_PANID_COMP  (1 << 6)
#define FRAME_CTRL_DST_ADDR_NONE (0x0 << 10)
#define FRAME_CTRL_DST_ADDR_SHORT (0x2 << 10)
#define FRAME_CTRL_DST_ADDR_EXT  (0x3 << 10)
#define FRAME_CTRL_DST_ADDR_MASK (0x3 << 10)
#define FRAME_CTRL_SRC_ADDR_NONE (0x0 << 14)
#define FRAME_CTRL_SRC_ADDR_SHORT (0x2 << 14)
#define FRAME_CTRL_SRC_ADDR_EXT  (0x3 << 14)
#define FRAME_CTRL_SRC_ADDR_MASK (0x3 << 14)
// Specific combinations
#define FRAME_CTRL_DATA_SHORTADDR_PANIDCOMP (FRAME_CTRL_TYPE_DATA | FRAME_CTRL_PANID_COMP | FRAME_CTRL_DST_ADDR_SHORT | FRAME_CTRL_SRC_ADDR_SHORT) // 0x8841
#define FRAME_CTRL_DATA_EXTADDR_PANIDCOMP   (FRAME_CTRL_TYPE_DATA | FRAME_CTRL_PANID_COMP | FRAME_CTRL_DST_ADDR_EXT | FRAME_CTRL_SRC_ADDR_EXT)     // 0xCC41

// Additional Register Definitions (Moved from .c)
#define RG_EUI          0x01    /* Extended Unique Identifier */
#define RG_SYS_TIME     0x06    /* System Time */
#define RG_DX_TIME      0x0A    /* Delayed Send/Receive Time */
#define RG_RX_FQUAL     0x12    /* Receive Frame Quality Information */
#define RG_AGC_CTRL     0x23    /* Automatic Gain Control */
#define RG_RF_CONF      0x28    /* RF Configuration */
#define RG_TX_CAL       0x2A    /* Transmitter Calibration */
#define RG_FS_CTRL      0x2B    /* Frequency Synthesizer Control */
#define RG_AON          0x2C    /* Always-On Control */
#define RG_OTP_IF       0x2D    /* OTP Memory Interface */
#define RG_LDE_IF       0x2E    /* Leading Edge Detection Interface */
#define RG_DMA_CONF     0x37    /* DMA Configuration */
#define RG_DMA_ADDR     0x38    /* DMA Address Configuration */
#define RG_DMA_STATUS   0x39    /* DMA Status */

// --- Common Sub-Addresses (Moved from .c) ---
#define SUB_PMSC_CTRL0  0x00    /* PMSC Control Register 0 */
#define SUB_PMSC_CTRL1  0x04    /* PMSC Control Register 1 */
#define SUB_SYS_STATUS_ALL 0x00  /* System Status Register All */

// --- System Event Flags (Moved from .c) ---
#define SYS_STATUS_RXFCG        0x00004000UL  // RX Frame Checksum Good (Bit 14)
#define SYS_STATUS_RXPHD        0x00001000UL  // RX PHY Header Detected (Bit 12)
#define SYS_STATUS_RXSFDD       0x00000200UL  // RX SFD Detected (Bit 9)
#define SYS_STATUS_RXPRD        0x00000100UL  // RX Preamble Detected (Bit 8)
#define SYS_STATUS_TXFRS        0x00000080UL  // TX Frame Sent (Bit 7)
#define SYS_STATUS_TXPHS        0x00000040UL  // TX PHY Header Sent (Bit 6)
#define SYS_STATUS_TXPRS        0x00000020UL  // TX Preamble Sent (Bit 5)
#define SYS_STATUS_LDEDONE      0x00000400UL  // LDE Processing Done (Bit 10)
#define SYS_STATUS_CLKPLL_LL    0x02000000UL  // Clock PLL Lock Lost (Bit 25)
#define SYS_STATUS_RFPLL_LL     0x01000000UL  // RF PLL Lock Lost (Bit 24)
#define SYS_STATUS_IRQS         (1UL << 0)    // IRQ Status (Overall flag? Check datasheet)
// Combined masks
#define SYS_STATUS_ALL_RX_GOOD  (SYS_STATUS_RXFCG | SYS_STATUS_RXPHD | SYS_STATUS_RXSFDD | SYS_STATUS_RXPRD)
#define SYS_STATUS_ALL_TX       (SYS_STATUS_TXFRS | SYS_STATUS_TXPHS | SYS_STATUS_TXPRS)
#define SYS_STATUS_ALL_RX_ERR   0x007F8000UL // Placeholder for RX Error flags (Need specific flags from datasheet, e.g., PHE, RSE, CRCE, SFDTO, PTO, FTO)
#define SYS_STATUS_ALL_ERR      (SYS_STATUS_ALL_RX_ERR | SYS_STATUS_CLKPLL_LL | SYS_STATUS_RFPLL_LL)
#define SYS_STATUS_ALL_DECA     0xFFFFFFFFUL // Not recommended, use specific masks

// Additional RX Status Flags
#define SYS_STATUS_RXDFR        0x00002000UL  // RX Data Frame Ready (Bit 13)
#define SYS_STATUS_RXFCE        0x00008000UL  // RX Frame Check Error (Bit 15)
#define SYS_STATUS_RXRFSL       0x00010000UL  // RX Reed-Solomon Frame Sync Loss (Bit 16)
#define SYS_STATUS_RXRFTO       0x00020000UL  // RX RF Timeout (Bit 17)
#define SYS_STATUS_RXPTO        0x00200000UL  // RX Preamble Timeout (Bit 21)


// --- System Control Register Bits (SYS_CTRL - 0x0D) (Moved from .c) ---
// #define SYS_CTRL_TXSTRT         (1 << 1) already defined
// #define SYS_CTRL_TRXOFF         (1 << 6) already defined
// #define SYS_CTRL_RXENAB         (1 << 8) already defined
#define SYS_CTRL_SFCST          (1 << 0) // Suppress Frame Check Sequence Transmission
#define SYS_CTRL_TXDLYS         (1 << 2) // TX Delayed Send
#define SYS_CTRL_RXDLYS         (1 << 9) // RX Delayed Receive
#define SYS_CTRL_RXWTE          (1 << 10)// RX Wait Timeout Enable
#define SYS_CTRL_RXWTOE         (1 << 13)// RX Wait Timeout Override Enable (Check datasheet)

// --- DMA Configuration Register Bits (RG_DMA_CONF - 0x37) (Moved from .c) ---
#define DMA_CONF_RXEN           (1 << 0) // Enable RX DMA
#define DMA_CONF_TXEN           (1 << 1) // Enable TX DMA
#define DMA_CONF_OFFSET_SHIFT   8        // Offset shift for DMA buffer offset
#define DMA_CONF_OFFSET_MASK    (0xFF << DMA_CONF_OFFSET_SHIFT) // Mask for DMA buffer offset

// --- Hybrid frame header definition (custom format) ---
struct dw1000_hybrid_hdr {
    u16 frame_ctrl;   // IEEE 802.15.4 frame control field
    u8 seq_num;       // Sequence number
    u16 pan_id;       // PAN ID
    u64 dest_addr;    // Destination address
    u64 src_addr;     // Source address
    u8 frame_type;    // Frame type (our custom field)
} __packed;

// --- Transmit frame structure ---
struct dw1000_tx_frame {
    u16 len;          // Frame length
    u8 wait_resp;     // Whether to wait for response
    u8 tx_mode;       // Transmit mode
    u8 data[0];       // Variable length data segment
} __packed;

// --- Sensing frame structure ---
struct dw1000_sensing_frame {
    u64 kernel_timestamp_ns;  // Kernel timestamp
    u16 data_len;             // Data length
    u8 status[4];             // RX Frame Quality (e.g., read from RG_RX_FQUAL, first 4 bytes)
    u8 seq_num;               // Sequence number
    u64 src_addr;             // Source address
    u8 data[];                // Variable length data segment
} __packed;

// Helper function to calculate sensing data entry size
static inline size_t dw1000_sensing_entry_size(size_t data_len)
{
    // Ensure proper alignment for the start of the next entry
    return ALIGN(offsetof(struct dw1000_sensing_frame, data) + data_len, sizeof(long));
}

// --- DMA receive descriptor ---
struct dw1000_rx_desc {
    struct list_head list;       // List node
    void *buffer;                // Virtual address
    dma_addr_t dma_addr;         // Physical address
    size_t buffer_size;          // Buffer size
    size_t data_len;             // Actual data length
    bool in_use;                 // Whether in use
    dma_cookie_t dma_cookie;
    struct dw1000_hybrid_priv *priv;
};

// --- Configuration structure definition ---
struct dw1000_hybrid_config {
    u8 channel;             // Channel selection (1-8)
    u32 spi_speed_hz;       // SPI clock frequency
    u8 prf;                 // Pulse repetition frequency
    u16 preamble_length;    // Preamble length
    u8 preamble_code;       // Preamble code
    u8 data_rate;           // Data rate
    bool smart_power;       // Smart power control
    u16 tx_power;           // Transmit power value
    u16 pan_id;             // PAN ID
    bool accept_bad_frames; // Whether to accept bad frames
    u64 default_dest_addr;  // Default destination address
    u32 irq_count;          // Interrupt count
    bool promiscuous;       // Promiscuous mode
    bool ranging_enabled;   // Ranging function enabled
    u16 sfd_timeout;        // SFD timeout
    u16 preamble_timeout;   // Preamble timeout
};

// --- Statistics structure definition ---
struct dw1000_hybrid_stats {
    u32 tx_packets;         // Transmit packets
    u32 rx_packets;         // Receive packets
    u32 tx_bytes;           // Transmit bytes
    u32 rx_bytes;           // Receive bytes
    u32 tx_errors;          // Transmit errors
    u32 rx_errors;          // Receive errors
    u32 tx_dma_errors;      // TX DMA errors
    u32 rx_dma_errors;      // RX DMA errors
    u32 rx_crc_errors;      // CRC errors
    u32 rx_sync_errors;     // Sync errors
    u32 rx_timeout_errors;  // Timeout errors
    u32 rx_dropped_ring;    // Ring buffer dropped
    u32 rx_dropped_nomem;   // Memory不足丢包
    u32 rx_dropped_other;   // Other reasons dropped
    u32 rx_dropped_unknown_type; // Unknown type dropped
    u32 rx_sensing_packets; // Sensing data packets
    u32 rx_sensing_bytes;   // Sensing data bytes
};

// --- TX descriptor structure ---
struct dw1000_tx_desc {
    void *buffer;                    // Virtual address
    dma_addr_t dma_addr;            // Physical address
    size_t buffer_size;             // Buffer size
    size_t data_len;                // Data length
    bool in_use;                    // Whether in use
    struct list_head list;          // List node
    dma_cookie_t dma_cookie;        // DMA cookie
    struct sk_buff *skb;            // Associated SKB
    struct dw1000_hybrid_priv *priv; // Pointer to private data
};

// --- Device private data structure ---
struct dw1000_hybrid_priv {
    // Basic device pointers
    struct spi_device *spi;              // SPI device pointer
    struct net_device *netdev;           // Network device pointer
    
    // Character device related
    struct cdev cdev;                    // Character device
    dev_t cdev_devt;                     // Character device number
    struct class *cdev_class;            // Character device class
    struct mutex ioctl_lock;             // IOCTL mutex
    
    // DMA
    struct dma_chan *tx_dma_chan;        // Transmit DMA channel
    struct dma_chan *rx_dma_chan;        // Receive DMA channel
    bool rx_dma_active;                  // Receive DMA active
    
    // Lock and synchronization
    spinlock_t config_lock;              // Configuration lock
    spinlock_t stats_lock;               // Statistics lock
    spinlock_t rx_desc_lock;             // Receive descriptor lock
    spinlock_t tx_desc_lock;             // TX descriptor lock
    struct mutex spi_reg_mutex;          // SPI register access mutex
    spinlock_t fifo_lock;                // kfifo access lock
    
    // GPIO control
    struct gpio_desc *irq_gpio;          // Interrupt GPIO
    int irq_num;                         // Interrupt number
    
    // Network device related
    struct napi_struct napi;             // NAPI structure
    
    // Configuration and status
    struct dw1000_hybrid_config config;  // Device configuration
    struct dw1000_hybrid_stats stats;    // Device statistics
    atomic_t tx_pending_count;           // Pending transmit count
    atomic_t tx_sequence_num;            // Transmit sequence number counter
    
    // Ring buffer (sensing data from character device interface) - kfifo based
    wait_queue_head_t read_wq;           // Read wait queue
    struct kfifo sensing_fifo;           // kfifo structure
    void *sensing_buffer;                // vmalloc allocated buffer pointer
    size_t sensing_buffer_size;          // Buffer size
    
    // Receive descriptor
    struct dw1000_rx_desc rx_desc[NUM_RX_DESC]; // Receive descriptor array
    struct list_head rx_free_list;       // Free descriptor list
    struct list_head rx_pending_list;    // Pending descriptor list

    // Transmit descriptor
    struct dw1000_tx_desc tx_desc[NUM_TX_DESC];  // TX descriptor array
    struct list_head tx_free_list;     // Free TX descriptors
    struct list_head tx_pending_list;  // Submitted TX descriptors to DMA

    // SPI communication buffer
    u8 spi_buf[8];                       // SPI transaction frame header buffer

    // DMA buffer size configuration
    size_t tx_dma_buf_size;
    size_t rx_dma_buf_size;
};

// Frame type identifier
#define DW1000_FRAME_TYPE_NET     (0) // Network frame (via netdev)
#define DW1000_FRAME_TYPE_SENSING (1) // Sensing data frame (via chardev->mmap)
#define DW1000_FRAME_TYPE_RANGING (2) // Ranging data frame (via chardev)
#define DW1000_FRAME_TYPE_CONFIG  (3) // Configuration command frame (via chardev)

// --- IOCTL Command Definitions ---
#define DW1000_IOC_MAGIC 'd'
#define DW1000_IOC_GET_CONFIG _IOR(DW1000_IOC_MAGIC, 1, struct dw1000_hybrid_config)
#define DW1000_IOC_SET_CONFIG _IOW(DW1000_IOC_MAGIC, 2, struct dw1000_hybrid_config)
#define DW1000_IOC_TX_TEST    _IOW(DW1000_IOC_MAGIC, 3, int)
#define DW1000_IOC_RX_TEST    _IOW(DW1000_IOC_MAGIC, 4, int)
#define DW1000_IOC_RESET      _IO(DW1000_IOC_MAGIC, 5)
#define DW1000_IOC_GET_STATS  _IOR(DW1000_IOC_MAGIC, 6, struct dw1000_hybrid_stats)
#define DW1000_IOC_SET_DEST_ADDR _IOW(DW1000_IOC_MAGIC, 7, u64)
#define DW1000_IOC_ACCEPT_BAD_FRAMES _IOW(DW1000_IOC_MAGIC, 8, bool)

// --- DMA Configuration ---
#define NUM_RX_DESC         (16)
#define RX_DESC_BUFFER_SIZE (2048)
#define NUM_TX_DESC         (16)
#define TX_DESC_BUFFER_SIZE (2048)

// --- Power Configuration Structure ---
struct power_config {
    u8 da;      // Digital Amplifier
    u8 mixer;   // Mixer Power
    u8 pa;      // Power Amplifier
};

// Transmit callback context structure
struct dw1000_tx_cb_context {
    struct dw1000_hybrid_priv *priv;
    dma_addr_t dma_handle;
    size_t len;
    struct sk_buff *skb;     // SKB pointer, used to release the network device layer
};

#endif /* __DW1000_GENERIC_H__ */ 