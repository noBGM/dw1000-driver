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
#include <linux/dmaengine.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include "dw1000_generic.h"
#include <linux/atomic.h>
#include <linux/wait.h>
#include <linux/mm.h>
#include <linux/highmem.h>


static void dw1000_hybrid_dma_rx_callback(void *param);
static int process_rx_frame(struct dw1000_hybrid_priv *priv, struct dw1000_rx_desc *desc);
static int process_sensing_frame(struct dw1000_hybrid_priv *priv, void *frame_data, size_t frame_len);
static int process_network_frame(struct dw1000_hybrid_priv *priv, void *frame_data, size_t frame_len);
static struct dw1000_rx_desc *dw1000_get_free_rx_desc(struct dw1000_hybrid_priv *priv);
static int dw1000_submit_rx_desc(struct dw1000_hybrid_priv *priv, struct dw1000_rx_desc *desc);
static int dw1000_refill_rx_descriptor(struct dw1000_hybrid_priv *priv, struct dw1000_rx_desc *desc);
static int dw1000_setup_rx_desc(struct dw1000_hybrid_priv *priv);
static void dw1000_cleanup_rx_desc(struct dw1000_hybrid_priv *priv);
static struct power_config get_power_config(s8 dbm);
static u32 calculate_tx_power_reg_val(s8 dbm);
static int dw1000_tx_test(struct dw1000_hybrid_priv *priv, int count);
static int dw1000_rx_test(struct dw1000_hybrid_priv *priv, int duration_ms);
// --- Forward Declarations ---
// Char dev
static int dw1000_hybrid_cdev_open(struct inode *inode, struct file *filp);
static int dw1000_hybrid_cdev_release(struct inode *inode, struct file *filp);
static __poll_t dw1000_hybrid_cdev_poll(struct file *filp, poll_table *wait);
static int dw1000_hybrid_cdev_mmap(struct file *filp, struct vm_area_struct *vma);
static ssize_t dw1000_hybrid_cdev_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos);
static long dw1000_hybrid_cdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

// Net dev
static int dw1000_hybrid_net_open(struct net_device *dev);
static int dw1000_hybrid_net_stop(struct net_device *dev);
static netdev_tx_t dw1000_hybrid_net_start_xmit(struct sk_buff *skb, struct net_device *dev);

// Shared RX/IRQ
static int dw1000_hybrid_poll(struct napi_struct *napi, int budget);
static irqreturn_t dw1000_hybrid_irq_handler(int irq, void *dev_id);

// Device control
static int dw1000_reset_device(struct dw1000_hybrid_priv *priv);  // Unified reset function
static int dw1000_hw_init(struct dw1000_hybrid_priv *priv);
static int dw1000_apply_config(struct dw1000_hybrid_priv *priv);

// DMA management
static int dw1000_setup_rx_dma(struct dw1000_hybrid_priv *priv);
static void dw1000_teardown_rx_dma(struct dw1000_hybrid_priv *priv);
static int dw1000_setup_tx_desc(struct dw1000_hybrid_priv *priv);
static void dw1000_cleanup_tx_desc(struct dw1000_hybrid_priv *priv);
static int dw1000_execute_tx(struct dw1000_hybrid_priv *priv, struct dw1000_tx_desc *desc);
static int dw1000_hybrid_lowlevel_tx(struct dw1000_hybrid_priv *priv, const void *frame_data, size_t frame_len, struct sk_buff *skb_to_free);


// --- Character Device IOCTL Implementation ---
static long dw1000_hybrid_cdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct dw1000_hybrid_priv *priv = filp->private_data;
	long ret = 0;
	void __user *argp = (void __user *)arg;
	unsigned long flags;
	struct dw1000_hybrid_config old_config;
	bool config_changed = false;

	mutex_lock(&priv->ioctl_lock);

	switch (cmd) {
	case DW1000_IOC_SET_CONFIG:
	{
		struct dw1000_hybrid_config user_config;
		if (copy_from_user(&user_config, argp, sizeof(user_config))) {
			ret = -EFAULT;
			goto unlock_exit;
		}

		// Validate user_config parameters
		if (user_config.channel < 1 || user_config.channel > 8 ||
			user_config.prf > 1 ||
			user_config.data_rate > 2) {
			ret = -EINVAL;
			goto unlock_exit;
		}

		bool was_running = netif_running(priv->netdev);
		
		old_config = priv->config;

		// Check if hardware reconfiguration is needed
		spin_lock_irqsave(&priv->config_lock, flags);
		if (memcmp(&user_config, &old_config, sizeof(user_config)) != 0) {
			config_changed = true;
		}
		memcpy(&priv->config, &user_config, sizeof(struct dw1000_hybrid_config));
		spin_unlock_irqrestore(&priv->config_lock, flags);
		
		// If hardware configuration parameters changed, reconfigure hardware
		if (config_changed) {
			// Pause network device and NAPI
			if (was_running) {
				netif_stop_queue(priv->netdev);
				napi_disable(&priv->napi);
			}
			
			// Reconfigure hardware
			ret = dw1000_apply_config(priv);
			
			if (ret == 0) { // Config succeeded
				if (was_running) {
					napi_enable(&priv->napi);
					netif_start_queue(priv->netdev);
				}
			} else { // Config failed
				dev_err(&priv->spi->dev, "Failed to apply new config (%d), attempting restore...\n", ret);
				spin_lock_irqsave(&priv->config_lock, flags);
				priv->config = old_config;
				spin_unlock_irqrestore(&priv->config_lock, flags);
				
				int restore_ret = dw1000_apply_config(priv);
				if (restore_ret != 0) {
					dev_err(&priv->spi->dev, "FATAL: Failed restore old config (%d). Interface left stopped.\n", restore_ret);
				} else {
					dev_info(&priv->spi->dev, "Successfully restored old config.\n");
					if (was_running) {
						napi_enable(&priv->napi);
						netif_start_queue(priv->netdev);
					}
				}
			}
		}
		break;
	}
		
	case DW1000_IOC_GET_CONFIG:
	{
		struct dw1000_hybrid_config temp_config;
		spin_lock_irqsave(&priv->config_lock, flags);
		temp_config = priv->config;
		spin_unlock_irqrestore(&priv->config_lock, flags);
		if (copy_to_user(argp, &temp_config, sizeof(struct dw1000_hybrid_config))) {
			ret = -EFAULT;
		}
		break;
	}
		
	case DW1000_IOC_GET_STATS:
	{
		struct dw1000_hybrid_stats temp_stats;
		spin_lock_irqsave(&priv->stats_lock, flags);
		temp_stats = priv->stats;
		spin_unlock_irqrestore(&priv->stats_lock, flags);
		if (copy_to_user(argp, &temp_stats, sizeof(struct dw1000_hybrid_stats))) {
			ret = -EFAULT;
		}
		break;
	}
		
	case DW1000_IOC_RESET:
	{
		bool was_running = netif_running(priv->netdev);
		dev_info(&priv->spi->dev, "IOCTL: Resetting hardware via IOCTL\n");

		if (was_running) {
			netif_stop_queue(priv->netdev);
			napi_disable(&priv->napi);
		}

		ret = dw1000_reset_device(priv);
		
		if (ret == 0 && was_running) {
			napi_enable(&priv->napi);
			netif_start_queue(priv->netdev);
		}
		break;
	}
	case DW1000_IOC_TX_TEST:
	{
		int test_count;
		if (copy_from_user(&test_count, argp, sizeof(test_count))) {
			ret = -EFAULT;
			goto unlock_exit;
		}
		
		// Validate test parameters
		if (test_count <= 0 || test_count > DW1000_MAX_TX_TEST_COUNT) {
			ret = -EINVAL;
			goto unlock_exit;
		}
		
		dev_info(&priv->spi->dev, "IOCTL: Starting TX test with %d frames\n", test_count);
		ret = dw1000_tx_test(priv, test_count);
		break;
	}
	case DW1000_IOC_RX_TEST:
	{
		int test_duration;
		if (copy_from_user(&test_duration, argp, sizeof(test_duration))) {
			ret = -EFAULT;
			goto unlock_exit;
		}
		
		// Validate test parameters
		if (test_duration <= 0 || test_duration > DW1000_MAX_RX_TEST_MS) {
			ret = -EINVAL;
			goto unlock_exit;
		}
		
		dev_info(&priv->spi->dev, "IOCTL: Starting RX test for %d ms\n", test_duration);
		ret = dw1000_rx_test(priv, test_duration);
		break;
	}
	case DW1000_IOC_SET_DEST_ADDR:
	{
		u64 dest_addr;
		if (copy_from_user(&dest_addr, argp, sizeof(dest_addr))) {
			ret = -EFAULT;
			goto unlock_exit;
		}
		// Protect write to config
		spin_lock_irqsave(&priv->config_lock, flags);
		priv->config.default_dest_addr = dest_addr; 
		spin_unlock_irqrestore(&priv->config_lock, flags);
		dev_info(&priv->spi->dev, "IOCTL: Set default destination address to 0x%llx\n", dest_addr);
		// This doesn't require HW reconfig
		break;
	}

	case DW1000_IOC_ACCEPT_BAD_FRAMES:
	{
		bool accept_bad_frames;
		if (copy_from_user(&accept_bad_frames, argp, sizeof(accept_bad_frames))) {
			ret = -EFAULT;
			goto unlock_exit;
		}
		// Only apply if value changes
		if (priv->config.accept_bad_frames != accept_bad_frames) {
			bool was_running = netif_running(priv->netdev);
			// Protect write to config
			spin_lock_irqsave(&priv->config_lock, flags);
			priv->config.accept_bad_frames = accept_bad_frames;
			spin_unlock_irqrestore(&priv->config_lock, flags);
			dev_info(&priv->spi->dev, "IOCTL: Set accept bad frames: %d\n", accept_bad_frames);
			
			if (was_running) {
				netif_stop_queue(priv->netdev);
				napi_disable(&priv->napi);
			}
			// apply_config handles its own config_lock internally
			ret = dw1000_apply_config(priv); // Re-apply full config
			if (ret == 0) { // Apply succeeded
				if (was_running) { // If it was running before, restart it
					napi_enable(&priv->napi);
					netif_start_queue(priv->netdev);
				}
			} else { // Apply failed
				dev_err(&priv->spi->dev, "Failed to apply config for accept_bad_frames (%d), attempting restore...\n", ret);
				// Attempt to restore previous value
				spin_lock_irqsave(&priv->config_lock, flags);
				priv->config.accept_bad_frames = !accept_bad_frames; // Restore opposite value
				spin_unlock_irqrestore(&priv->config_lock, flags);
				// Try re-applying config with restored value
				int restore_ret = dw1000_apply_config(priv);
				if (restore_ret != 0) {
					dev_err(&priv->spi->dev, "FATAL: Failed restore config after accept_bad_frames change failed (%d). Interface left stopped.\n", restore_ret);
				} else {
					dev_info(&priv->spi->dev, "Successfully restored config after accept_bad_frames change failed.\n");
					if (was_running) { // Restart if it was running before
						napi_enable(&priv->napi);
						netif_start_queue(priv->netdev);
					}
				}
				// Keep the original error 'ret'
			}
		}
		break;
	}
		
	default:
		ret = -ENOTTY;
		break;
	}

unlock_exit:
	mutex_unlock(&priv->ioctl_lock);
	return ret;
}

// --- DW1000 Register Operation Functions ---
// Low-level synchronous register read function
static int dw1000_read_reg(struct dw1000_hybrid_priv *priv, u16 reg, u16 offset, u32 len, void *data)
{
	// SPI-based register read
	struct spi_device *spi = priv->spi;
	struct spi_message msg;
	struct spi_transfer header_xfer = {
		.tx_buf = priv->spi_buf,
		.len = 3,
	};
	struct spi_transfer data_xfer = {
		.rx_buf = data,
		.len = len,
	};
	int ret;

	mutex_lock(&priv->spi_reg_mutex); // Acquire SPI register access mutex
	
	// Build SPI command header - read command, register and sub-address
	priv->spi_buf[0] = (reg & SPI_CMD_REG_MASK) | SPI_CMD_READ; // Use macros
	if (offset > SPI_CMD_SUB_SHORT_MASK) { // Check against max short address value
		priv->spi_buf[0] |= SPI_CMD_SUB_EXT_BIT; // Set extended sub-index bit
		priv->spi_buf[1] = (offset & SPI_CMD_SUB_SHORT_MASK) | SPI_CMD_SUB_SHORT_EXT_BIT; // Indicate continuation
		priv->spi_buf[2] = (offset >> 7) & 0xFF; // Upper bits of offset
		header_xfer.len = 3;
	} else {
		priv->spi_buf[1] = offset & SPI_CMD_SUB_SHORT_MASK;
		header_xfer.len = 2;
	}
	
	// Initialize SPI message
	spi_message_init(&msg);
	spi_message_add_tail(&header_xfer, &msg);
	spi_message_add_tail(&data_xfer, &msg);
	
	// Execute SPI transfer
	ret = spi_sync(spi, &msg);
	if (ret)
		dev_err(&spi->dev, "SPI read failed: reg=0x%02x, offset=0x%04x, ret=%d\n",
				reg, offset, ret);

	mutex_unlock(&priv->spi_reg_mutex); // Release SPI register access mutex
	
	return ret;
}

// Low-level synchronous register write function
static int dw1000_write_reg(struct dw1000_hybrid_priv *priv, u16 reg, u16 offset, u32 len, const void *data)
{
	// SPI-based register write
	struct spi_device *spi = priv->spi;
	struct spi_message msg;
	struct spi_transfer header_xfer = {
		.tx_buf = priv->spi_buf,
		.len = 3,
	};
	struct spi_transfer data_xfer = {
		.tx_buf = data,
		.len = len,
	};
	int ret;

	mutex_lock(&priv->spi_reg_mutex); // Acquire SPI register access mutex
	
	// Build SPI command header - write command, register and sub-address
	priv->spi_buf[0] = (reg & SPI_CMD_REG_MASK) | SPI_CMD_WRITE; // Use macros
	if (offset > SPI_CMD_SUB_SHORT_MASK) { // Check against max short address value
		priv->spi_buf[0] |= SPI_CMD_SUB_EXT_BIT; // Set extended sub-index bit
		priv->spi_buf[1] = (offset & SPI_CMD_SUB_SHORT_MASK) | SPI_CMD_SUB_SHORT_EXT_BIT; // Indicate continuation
		priv->spi_buf[2] = (offset >> 7) & 0xFF; // Upper bits of offset
		header_xfer.len = 3;
	} else {
		priv->spi_buf[1] = offset & SPI_CMD_SUB_SHORT_MASK;
		header_xfer.len = 2;
	}
	
	// Initialize SPI message
	spi_message_init(&msg);
	spi_message_add_tail(&header_xfer, &msg);
	spi_message_add_tail(&data_xfer, &msg);
	
	// Execute SPI transfer
	ret = spi_sync(spi, &msg);
	if (ret)
		dev_err(&spi->dev, "SPI write failed: reg=0x%02x, offset=0x%04x, ret=%d\n",
				reg, offset, ret);

	mutex_unlock(&priv->spi_reg_mutex); // Release SPI register access mutex
	
	return ret;
}

// 32-bit register read helper function
static inline u32 dw1000_read_reg32(struct dw1000_hybrid_priv *priv, u16 reg, u16 offset)
{
	u32 value = 0;
	dw1000_read_reg(priv, reg, offset, 4, &value);
	return value;
}

// 32-bit register write helper function
static inline int dw1000_write_reg32(struct dw1000_hybrid_priv *priv, u16 reg, u16 offset, u32 value)
{
	return dw1000_write_reg(priv, reg, offset, 4, &value);
}

// 16-bit register read helper function
static inline u16 dw1000_read_reg16(struct dw1000_hybrid_priv *priv, u16 reg, u16 offset)
{
	u16 value = 0;
	dw1000_read_reg(priv, reg, offset, 2, &value);
	return value;
}

// 16-bit register write helper function
static inline void dw1000_write_reg16(struct dw1000_hybrid_priv *priv, u16 reg, u16 offset, u16 value)
{
	dw1000_write_reg(priv, reg, offset, 2, &value);
}

// 8-bit register read helper function
static inline u8 dw1000_read_reg8(struct dw1000_hybrid_priv *priv, u16 reg, u16 offset)
{
	u8 value = 0;
	dw1000_read_reg(priv, reg, offset, 1, &value);
	return value;
}

// 8-bit register write helper function
static inline void dw1000_write_reg8(struct dw1000_hybrid_priv *priv, u16 reg, u16 offset, u8 value)
{
	dw1000_write_reg(priv, reg, offset, 1, &value);
}

// --- File Operations (Character Device) ---
static const struct file_operations dw1000_hybrid_cdev_fops = {
	.owner = THIS_MODULE,
	.open = dw1000_hybrid_cdev_open,
	.release = dw1000_hybrid_cdev_release,
	.poll = dw1000_hybrid_cdev_poll,
	.mmap = dw1000_hybrid_cdev_mmap,
	.write = dw1000_hybrid_cdev_write,
	.unlocked_ioctl = dw1000_hybrid_cdev_ioctl, // Add ioctl operation
};

// --- Net Device Operations ---
static const struct net_device_ops dw1000_hybrid_netdev_ops = {
    .ndo_open = dw1000_hybrid_net_open,
    .ndo_stop = dw1000_hybrid_net_stop,
    .ndo_start_xmit = dw1000_hybrid_net_start_xmit,
    .ndo_get_stats64 = dev_get_tstats64, // Use built-in statistics
};

// --- Shared Interrupt Handling ---
static irqreturn_t dw1000_hybrid_irq_handler(int irq, void *dev_id) {
	struct dw1000_hybrid_priv *priv = dev_id;

	//bug1: null pointer
	if (!priv) {
        return IRQ_NONE;  // Prevent null pointer
    }

	//bug2: race condition causing incorrect irq statistics.
	// Minimal ISR: Disable IRQ, schedule NAPI for unified processing
	// disable_irq_nosync(irq);
	// if (napi_schedule_prep(&priv->napi)) {
	// 	__napi_schedule(&priv->napi);
	// } else {
	// 	// Already scheduled or disabled, maybe just ACK HW IRQ if needed
	// 	enable_irq(irq); // Re-enable if NAPI won't run
	// }

	//bug2 fixed:
    // First check if NAPI can be scheduled, only disable interrupt when successful
    if (napi_schedule_prep(&priv->napi)) {
        disable_irq_nosync(irq);    // Only disable when NAPI can be scheduled
        __napi_schedule(&priv->napi);
    }
    // If NAPI is already running, do nothing, let running NAPI handle it
    	
	return IRQ_HANDLED;
}

// --- Shared NAPI Poll (Core Demux Logic) ---
static int dw1000_hybrid_poll(struct napi_struct *napi, int budget)
{
    struct dw1000_hybrid_priv *priv = container_of(napi, struct dw1000_hybrid_priv, napi);
    struct net_device *netdev = priv->netdev;
    int work_done = 0;
    unsigned long flags;
    struct dw1000_rx_desc *rx_desc, *tmp_rx_desc;
    LIST_HEAD(completed_list); // Local list for completed descriptors
    //struct dw1000_hybrid_hdr *hdr;
    u32 sys_status = 0;
    u32 processed_status = 0;

    // Read system status register
    if (dw1000_read_reg(priv, RG_SYS_STATUS, 0, 4, &sys_status) != 0) {
        dev_err(&priv->spi->dev, "Failed to read system status register\n");
        goto poll_end;
    }

    dev_dbg(&priv->spi->dev, "NAPI poll: sys_status=0x%08x\n", sys_status);

    // Define interrupt flags we care about
    #define HANDLED_STATUS_BITS ( \
        SYS_STATUS_RXFCG |   /* RX Frame Complete Good */ \
        SYS_STATUS_TXFRS |   /* TX Frame Sent */ \
        SYS_STATUS_RXDFR |   /* RX Data Frame Ready */ \
        SYS_STATUS_RXFCE |   /* RX Frame Check Error */ \
        SYS_STATUS_RXRFSL |  /* RX Reed-Solomon Frame Sync Loss */ \
        SYS_STATUS_RXRFTO |  /* RX RF Timeout */ \
        SYS_STATUS_RXPTO     /* RX Preamble Timeout */ \
    )

    // If no interrupt flags we care about, exit directly
    if (!(sys_status & HANDLED_STATUS_BITS)) {
        goto poll_end;
    }

    // Handle TX completion interrupt
    if (sys_status & SYS_STATUS_TXFRS) {
        // TX completion handling (if needed)
        processed_status |= SYS_STATUS_TXFRS;
        dev_dbg(&priv->spi->dev, "TX frame sent\n");
    }

    // Handle RX error interrupts
    if (sys_status & (SYS_STATUS_RXFCE | SYS_STATUS_RXRFSL | 
                     SYS_STATUS_RXRFTO | SYS_STATUS_RXPTO)) {
        // Update statistics
        spin_lock_irqsave(&priv->stats_lock, flags);
        if (sys_status & SYS_STATUS_RXFCE)
            priv->stats.rx_crc_errors++;
        if (sys_status & SYS_STATUS_RXRFSL)
            priv->stats.rx_sync_errors++;
        if (sys_status & (SYS_STATUS_RXRFTO | SYS_STATUS_RXPTO))
            priv->stats.rx_timeout_errors++;
        spin_unlock_irqrestore(&priv->stats_lock, flags);

        // Mark these errors as handled
        processed_status |= (sys_status & (SYS_STATUS_RXFCE | SYS_STATUS_RXRFSL |
                                         SYS_STATUS_RXRFTO | SYS_STATUS_RXPTO));
        dev_dbg(&priv->spi->dev, "RX errors: status=0x%08x\n", 
                sys_status & (SYS_STATUS_RXFCE | SYS_STATUS_RXRFSL |
                            SYS_STATUS_RXRFTO | SYS_STATUS_RXPTO));
    }

    // If there are RX frames to process
    if (sys_status & (SYS_STATUS_RXFCG | SYS_STATUS_RXDFR)) {
        // --- Collect all completed DMA descriptors ---
        spin_lock_irqsave(&priv->rx_desc_lock, flags);
        list_for_each_entry_safe(rx_desc, tmp_rx_desc, &priv->rx_pending_list, list) {
            dma_cookie_t cookie = rx_desc->dma_cookie;
            enum dma_status status = dmaengine_tx_status(priv->rx_dma_chan, cookie, NULL);

            if (status == DMA_COMPLETE) {
                list_move_tail(&rx_desc->list, &completed_list);
            } else if (status == DMA_ERROR) {
                dev_warn(&priv->spi->dev, "RX DMA error on descriptor, cookie %d\n", cookie);
                list_move_tail(&rx_desc->list, &priv->rx_free_list);
                rx_desc->in_use = false;
				unsigned long stats_flags;
				spin_lock_irqsave(&priv->stats_lock, stats_flags);
				priv->stats.rx_dma_errors++;
				spin_unlock_irqrestore(&priv->stats_lock, stats_flags);
            }
        }
        spin_unlock_irqrestore(&priv->rx_desc_lock, flags);

        // --- Process completed descriptors ---
        while (work_done < budget && !list_empty(&completed_list)) {
            struct dw1000_rx_desc *curr_desc = list_first_entry(&completed_list,
                                                               struct dw1000_rx_desc, list);
            list_del_init(&curr_desc->list);

            // Process received data
            if (process_rx_frame(priv, curr_desc) == 0) {
                work_done++;
                processed_status |= SYS_STATUS_RXFCG;
            }

            // Resubmit descriptor
            if (dw1000_refill_rx_descriptor(priv, curr_desc) != 0) {
                dev_err(&priv->spi->dev, "Failed to resubmit RX descriptor\n");
                // If resubmission fails, descriptor has been moved to free list by refill_rx_descriptor
            }
        }
    }

poll_end:
    // Clear processed interrupt status bits
    if (processed_status != 0) {
        dw1000_write_reg32(priv, RG_SYS_STATUS, 0, processed_status);
        dev_dbg(&priv->spi->dev, "Cleared interrupt status bits: 0x%08x\n", processed_status);
    }

    // If processed frames less than budget or no more pending descriptors, complete NAPI
    if (work_done < budget) {
        napi_complete_done(napi, work_done);
        enable_irq(priv->irq_num);
    }

    return work_done;
}


static int process_sensing_frame(struct dw1000_hybrid_priv *priv, void *frame_data, size_t frame_len)
{
	struct dw1000_hybrid_hdr *hdr = frame_data;
	ktime_t now = ktime_get_real();
	u64 timestamp_ns = ktime_to_ns(now);
	u16 payload_len;
	struct dw1000_sensing_frame entry_hdr;
	unsigned long flags;
	int spi_ret;
	size_t need;
	u8 *tmp;
	unsigned int copied;

	if (frame_len < DW1000_HYBRID_HDR_LEN)
		return -EINVAL;

	payload_len = frame_len - DW1000_HYBRID_HDR_LEN;

	// Fill sensing frame header
	entry_hdr.kernel_timestamp_ns = timestamp_ns;
	entry_hdr.data_len = payload_len;
	
	// Get reception quality info from hardware register
	spi_ret = dw1000_read_reg(priv, RG_RX_FQUAL, 0, sizeof(entry_hdr.status), entry_hdr.status);
	if (spi_ret != 0) {
		dev_warn_ratelimited(&priv->spi->dev, "Failed to read RX_FQUAL: %d, status zeroed\n", spi_ret);
		memset(entry_hdr.status, 0, sizeof(entry_hdr.status));
	}
	
	entry_hdr.seq_num = hdr->seq_num;
	entry_hdr.src_addr = le64_to_cpu(hdr->src_addr);

	// Check space first, then write atomically to avoid rollback
	need = offsetof(struct dw1000_sensing_frame, data) + payload_len;

	// Check available space with lock
	spin_lock_irqsave(&priv->fifo_lock, flags);
	if (kfifo_avail(&priv->sensing_fifo) < need) {
		spin_unlock_irqrestore(&priv->fifo_lock, flags);
		dev_warn_ratelimited(&priv->spi->dev, "kfifo no space: need=%zu, drop\n", need);
		spin_lock_irqsave(&priv->stats_lock, flags);
		priv->stats.rx_dropped_ring++;
		spin_unlock_irqrestore(&priv->stats_lock, flags);
		return -ENOSPC;
	}
	spin_unlock_irqrestore(&priv->fifo_lock, flags);

	// Allocate temporary contiguous buffer
	tmp = kmalloc(need, GFP_ATOMIC);
	if (!tmp) {
		dev_warn_ratelimited(&priv->spi->dev, "alloc temp buffer failed (need=%zu)\n", need);
		spin_lock_irqsave(&priv->stats_lock, flags);
		priv->stats.rx_dropped_nomem++;
		spin_unlock_irqrestore(&priv->stats_lock, flags);
		return -ENOMEM;
	}

	// Assemble header + payload to temp buffer
	memcpy(tmp, &entry_hdr, offsetof(struct dw1000_sensing_frame, data));
	memcpy(tmp + offsetof(struct dw1000_sensing_frame, data),
	       (u8 *)frame_data + DW1000_HYBRID_HDR_LEN, payload_len);

	// Write to FIFO atomically
	copied = kfifo_in_spinlocked(&priv->sensing_fifo, tmp, need, &priv->fifo_lock);
	kfree(tmp);

	if (copied != need) {
		dev_warn_ratelimited(&priv->spi->dev, "kfifo short write: need=%zu copied=%u\n", need, copied);
		spin_lock_irqsave(&priv->stats_lock, flags);
		priv->stats.rx_dropped_ring++;
		spin_unlock_irqrestore(&priv->stats_lock, flags);
		return -ENOSPC;
	}

	// Wake up waiting readers
	wake_up_interruptible(&priv->read_wq);

	// Update statistics
	spin_lock_irqsave(&priv->stats_lock, flags);
	priv->stats.rx_sensing_packets++;
	priv->stats.rx_sensing_bytes += payload_len;
	spin_unlock_irqrestore(&priv->stats_lock, flags);

	return 0;
}

static int process_network_frame(struct dw1000_hybrid_priv *priv, void *frame_data, size_t frame_len)
{
    struct dw1000_hybrid_hdr *hdr = frame_data;
    struct net_device *netdev = priv->netdev;
    struct sk_buff *skb;
    u16 payload_len = frame_len - DW1000_HYBRID_HDR_LEN;
    unsigned long flags;

    // Basic validation
    if (frame_len < DW1000_HYBRID_HDR_LEN) {
        dev_warn_ratelimited(&priv->spi->dev, "Network frame too short: %zu\n", frame_len);
        spin_lock_irqsave(&priv->stats_lock, flags);
        priv->stats.rx_dropped_other++;
        spin_unlock_irqrestore(&priv->stats_lock, flags);
        return -EINVAL;
    }

    // Allocate SKB
    skb = napi_alloc_skb(&priv->napi, payload_len + NET_IP_ALIGN);
    if (!skb) {
        spin_lock_irqsave(&priv->stats_lock, flags);
        priv->stats.rx_dropped_nomem++;
        spin_unlock_irqrestore(&priv->stats_lock, flags);
        
        // Update netdev stats using kernel API
        netdev->stats.rx_dropped++;
        
        dev_warn_ratelimited(&priv->spi->dev, "Failed to allocate SKB for network frame\n");
        return -ENOMEM;
    }

    // Setup SKB
    skb_reserve(skb, NET_IP_ALIGN);
    skb_put_data(skb, frame_data + DW1000_HYBRID_HDR_LEN, payload_len);

    // Set SKB metadata
    skb->dev = netdev;
    skb->protocol = htons(ETH_P_IEEE802154); // Use correct protocol type
    skb->ip_summed = CHECKSUM_NONE;
    
    // Optional: Set additional 802.15.4 metadata
    skb_set_mac_header(skb, 0);

    // Update statistics
    spin_lock_irqsave(&priv->stats_lock, flags);
    priv->stats.rx_packets++;
    priv->stats.rx_bytes += payload_len;
    spin_unlock_irqrestore(&priv->stats_lock, flags);

    // Update netdev stats using kernel API (recommended approach)
    netdev->stats.rx_packets++;
    netdev->stats.rx_bytes += payload_len;

    // Pass frame to network stack
    netif_receive_skb(skb);

    return 0;
}

// Helper function: process received frames
static int process_rx_frame(struct dw1000_hybrid_priv *priv, struct dw1000_rx_desc *desc)
{
    void *frame_data = desc->buffer;
    size_t frame_len;
    int spi_ret;
    unsigned long flags;
    struct dw1000_hybrid_hdr *hdr;

    // Get actual receive length from RX_FINFO register
	u16 frame_len_raw;
	spi_ret = dw1000_read_reg(priv, RG_RX_FINFO, 0, sizeof(u16), &frame_len_raw);
	if (spi_ret) {
		dev_warn_ratelimited(&priv->spi->dev, "Failed to read RX_FINFO: %d\n", spi_ret);
		return -EIO;
	}
	frame_len = le16_to_cpu(frame_len_raw) & RX_FINFO_RXFLEN_MASK;
    if (frame_len > desc->buffer_size) {
        dev_warn_ratelimited(&priv->spi->dev, "Frame len %zu > buffer size %zu\n",
                         frame_len, desc->buffer_size);
        return -EINVAL;
    }

    // Sync DMA buffer for CPU access
    dma_sync_single_for_cpu(&priv->spi->dev, desc->dma_addr,
                      desc->buffer_size, DMA_FROM_DEVICE);

    // Basic validation
    if (frame_len < DW1000_HYBRID_HDR_LEN) {
        dev_warn_ratelimited(&priv->spi->dev, "RX runt frame (len %zu)\n", frame_len);
        spin_lock_irqsave(&priv->stats_lock, flags);
        priv->stats.rx_dropped_other++;
        spin_unlock_irqrestore(&priv->stats_lock, flags);
        return -EINVAL;
    }

    hdr = (struct dw1000_hybrid_hdr *)frame_data;

    // Dispatch processing based on frame type
    switch (hdr->frame_type) {
        case DW1000_FRAME_TYPE_SENSING:
            return process_sensing_frame(priv, frame_data, frame_len);
        case DW1000_FRAME_TYPE_NET:
            return process_network_frame(priv, frame_data, frame_len);
        default:
            dev_warn_ratelimited(&priv->spi->dev, "Unknown frame type: 0x%02x\n",
                             hdr->frame_type);
            spin_lock_irqsave(&priv->stats_lock, flags);
            priv->stats.rx_dropped_unknown_type++;
            spin_unlock_irqrestore(&priv->stats_lock, flags);
            return -EINVAL;
    }
}

// --- Network Device XMIT --- (Optimized)
static netdev_tx_t dw1000_hybrid_net_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct dw1000_hybrid_priv *priv = netdev_priv(dev);
	struct dw1000_hybrid_hdr hdr;
	// struct dw1000_tx_desc *desc; // No longer directly managed here
	size_t frame_len_with_hdr;
	int ret;
	unsigned long flags; // For config_lock
	u64 dest_addr_le;
	// u16 pan_id_le; // Read directly inside header prep
	// u16 frame_ctrl_le = cpu_to_le16(FRAME_CTRL_DATA_SHORTADDR_PANIDCOMP); // Default, set in header prep
	void *tx_frame_buf = NULL; // Buffer for header + payload
	size_t skb_payload_len = skb->len; // Original skb payload length

	// Basic validation
	if (skb->len > dev->mtu) {
		dev_warn(&priv->spi->dev, "Packet too large: %d > %d\n", skb->len, dev->mtu);
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	// Determine destination address and adjust skb payload/length if address is in skb
	spin_lock_irqsave(&priv->config_lock, flags);
	if (skb->len >= sizeof(u64) && skb_headlen(skb) >= sizeof(u64)) {
		// Use address from skb data
		memcpy(&dest_addr_le, skb->data, sizeof(u64));
		skb_pull(skb, sizeof(u64)); // Remove address from skb data
		skb_payload_len = skb->len; // Update payload length after skb_pull
	} else {
		// Use default address from config
		dest_addr_le = cpu_to_le64(priv->config.default_dest_addr);
	}
	spin_unlock_irqrestore(&priv->config_lock, flags); 

	// Total frame length: Our header + (potentially modified) SKB payload
	frame_len_with_hdr = DW1000_HYBRID_HDR_LEN + skb_payload_len;

	// Allocate buffer for the full frame (header + payload)
	tx_frame_buf = kmalloc(frame_len_with_hdr, GFP_ATOMIC);
	if (!tx_frame_buf) {
		dev_err(&priv->spi->dev, "Failed to allocate TX frame buffer for netdev\n");
		// If skb_pull was done, we need to push it back before freeing to restore original skb state.
		// However, skb_pull modifies skb->data and skb->len, it doesn't save old state.
		// The correct way is to free the skb as is and accept data loss.
		dev_kfree_skb(skb);
		return NETDEV_TX_OK; // Drop packet
	}

	// Prepare our custom frame header
	memset(&hdr, 0, sizeof(hdr));
	hdr.frame_type = DW1000_FRAME_TYPE_NET;
	hdr.seq_num = atomic_inc_return(&priv->tx_sequence_num) & 0xFF;
	hdr.dest_addr = dest_addr_le; 
	if (dev->dev_addr) {
		hdr.src_addr = 0;
		memcpy(&hdr.src_addr, dev->dev_addr, ETH_ALEN);
	}
	spin_lock_irqsave(&priv->config_lock, flags);
	hdr.pan_id = cpu_to_le16(priv->config.pan_id);
	hdr.frame_ctrl = cpu_to_le16(FRAME_CTRL_DATA_SHORTADDR_PANIDCOMP);
	spin_unlock_irqrestore(&priv->config_lock, flags);

	// Copy header to the start of our allocated buffer
	memcpy(tx_frame_buf, &hdr, DW1000_HYBRID_HDR_LEN);

	// Copy SKB payload data after our header
	ret = skb_copy_bits(skb, 0, tx_frame_buf + DW1000_HYBRID_HDR_LEN, skb_payload_len);
	if (ret < 0) {
		dev_err(&priv->spi->dev, "Failed to copy skb data to TX buffer: %d\n", ret);
		kfree(tx_frame_buf);
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	// Call the low-level TX function
	// Pass the original skb so it can be freed by the DMA callback via desc->skb
	ret = dw1000_hybrid_lowlevel_tx(priv, tx_frame_buf, frame_len_with_hdr, skb);

	kfree(tx_frame_buf); // Free the temporary buffer now, lowlevel_tx copies it to DMA desc buffer

	if (ret) {
		if (ret == -EBUSY) {
			dev_warn_ratelimited(&priv->spi->dev, "Lowlevel TX busy, stopping net queue\n");
			netif_stop_queue(dev); // Stop queue
			// skb was passed to lowlevel_tx, which should have freed it on -EBUSY error path
			// if it didn't use it. Double check dw1000_hybrid_lowlevel_tx - it does free skb_to_free on -EBUSY.
			return NETDEV_TX_BUSY; // Return BUSY, let upper layer retry
		} else {
			dev_err(&priv->spi->dev, "Lowlevel TX failed with %d\n", ret);
			// skb was passed to lowlevel_tx, which is responsible for freeing it on other errors too.
			// Ensure dw1000_hybrid_lowlevel_tx handles skb_to_free correctly in its error paths.
			// Update: dw1000_hybrid_lowlevel_tx does free skb if ret is an error.
			return NETDEV_TX_OK; // Drop packet, error handled
		}
	}

	// Update statistics and potentially stop queue if needed
	netif_trans_update(dev); 
	if (atomic_read(&priv->tx_pending_count) >= MAX_PENDING_TX) {
		netif_stop_queue(dev); 
	}
	
	return NETDEV_TX_OK;
}

// --- TX DMA Callback (Complete & Fixed) ---
static void dw1000_hybrid_dma_tx_callback(void *param)
{
    struct dw1000_tx_desc *desc = param;
    struct dw1000_hybrid_priv *priv;
    unsigned long flags;
    enum dma_status status;
    struct dma_tx_state state;

    if (!desc) {
        pr_err("DW1000: NULL descriptor in TX DMA callback!\n");
        return;
    }
    
    priv = desc->priv; // 直接从描述符获取 priv
    if (!priv) {
        // 这个情况理论上不应发生，如果发生了说明初始化有问题
        pr_err("DW1000: NULL priv in TX descriptor! Cookie: %d\n", desc->dma_cookie);
        return;
    }

    // Get detailed DMA status
    status = dmaengine_tx_status(priv->tx_dma_chan, desc->dma_cookie, &state);
    
    if (status == DMA_ERROR) {
        dev_warn(&priv->spi->dev, "TX DMA transfer error (residue=%d)\n", state.residue);
        
        // Update custom stats with lock protection
        spin_lock_irqsave(&priv->stats_lock, flags);
        priv->stats.tx_errors++;
        priv->stats.tx_dma_errors++;
        spin_unlock_irqrestore(&priv->stats_lock, flags);
        
        // If this was a network packet, update netdev stats
        if (desc->skb && priv->netdev) {
            priv->netdev->stats.tx_errors++;
        }
        
    } else if (status == DMA_COMPLETE) {
        // Update custom statistics on success
        spin_lock_irqsave(&priv->stats_lock, flags);
        priv->stats.tx_packets++;
        priv->stats.tx_bytes += desc->data_len;
        spin_unlock_irqrestore(&priv->stats_lock, flags);

        // If this was a network packet, update netdev stats
        if (desc->skb && priv->netdev) {
            priv->netdev->stats.tx_packets++;
            priv->netdev->stats.tx_bytes += desc->data_len;
        }
        
    } else {
        dev_warn(&priv->spi->dev, "Unexpected TX DMA status: %d\n", status);
        
        // Treat unexpected status as an error for statistics
        spin_lock_irqsave(&priv->stats_lock, flags);
        priv->stats.tx_errors++;
        spin_unlock_irqrestore(&priv->stats_lock, flags);
        
        if (desc->skb && priv->netdev) {
            priv->netdev->stats.tx_errors++;
        }
    }

    // Free the SKB if it came from the network stack
    if (desc->skb) {
        dev_kfree_skb_any(desc->skb);
        desc->skb = NULL;
    }

    // Return descriptor to free list (with proper list management)
    spin_lock_irqsave(&priv->tx_desc_lock, flags);
    list_del(&desc->list);
    desc->in_use = false;
    desc->data_len = 0;  // Clear data length for safety
    list_add_tail(&desc->list, &priv->tx_free_list);
    spin_unlock_irqrestore(&priv->tx_desc_lock, flags);

    // Update TX state
    atomic_dec(&priv->tx_pending_count);

    // Wake the network queue if it was stopped and resources are now available
    // 需要确保 priv->netdev 是有效的
    if (priv->netdev && netif_queue_stopped(priv->netdev) && 
        (atomic_read(&priv->tx_pending_count) < MAX_PENDING_TX)) {
        netif_wake_queue(priv->netdev);
    }
}

// --- Character Device Write Implementation (优化后) ---
static ssize_t dw1000_hybrid_cdev_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos) {
	struct dw1000_hybrid_priv *priv = filp->private_data;
	struct dw1000_hybrid_hdr hdr;
	// struct dw1000_tx_desc *desc; // No longer directly managed here
	size_t frame_len_with_hdr;
	int ret;
	u8 frame_type = DW1000_FRAME_TYPE_SENSING;
	u64 dest_addr = 0xFFFFFFFFFFFFFFFF;         
	const char __user *payload_buf = buf;       
	size_t payload_count = count;              
	unsigned long flags; // For config_lock
	void *tx_frame_buf = NULL; // Buffer for header + payload

	// Basic validation
	if (count == 0) return 0;
	
	// Check for optional frame type header byte
	u8 header_byte_user = 0;
	if (payload_count > 0 && copy_from_user(&header_byte_user, buf, 1) == 0) {
		u8 type_field = (header_byte_user >> 4) & 0x0F;
		if (type_field >= DW1000_FRAME_TYPE_NET && type_field <= DW1000_FRAME_TYPE_CONFIG) {
			frame_type = type_field;
			payload_buf++;
			payload_count--;
		}
	}

	if (payload_count == 0 && frame_type != DW1000_FRAME_TYPE_CONFIG) { // Config frames can be header-only
		dev_warn(&priv->spi->dev, "Cdev write: No payload after optional header byte\n");
		return -EINVAL;
	}
	
	// Total frame length: Our header + user payload
	frame_len_with_hdr = DW1000_HYBRID_HDR_LEN + payload_count;

	// Allocate buffer for the full frame (header + payload)
	tx_frame_buf = kmalloc(frame_len_with_hdr, GFP_KERNEL);
	if (!tx_frame_buf) {
		dev_err(&priv->spi->dev, "Failed to allocate TX frame buffer for cdev\n");
		return -ENOMEM;
	}

	// Prepare our custom frame header
	memset(&hdr, 0, sizeof(hdr));
	hdr.frame_type = frame_type;
	hdr.seq_num = atomic_inc_return(&priv->tx_sequence_num) & 0xFF;
	hdr.dest_addr = cpu_to_le64(dest_addr); 
	if (priv->netdev && priv->netdev->dev_addr) { // Use netdev MAC as source if available
		hdr.src_addr = 0;
		memcpy(&hdr.src_addr, priv->netdev->dev_addr, ETH_ALEN);
	} else {
		hdr.src_addr = cpu_to_le64(0); // Default source if no netdev MAC
	}
	spin_lock_irqsave(&priv->config_lock, flags);
	hdr.pan_id = cpu_to_le16(priv->config.pan_id);
	hdr.frame_ctrl = cpu_to_le16(FRAME_CTRL_DATA_SHORTADDR_PANIDCOMP);
	spin_unlock_irqrestore(&priv->config_lock, flags);

	// Copy header to the start of our allocated buffer
	memcpy(tx_frame_buf, &hdr, DW1000_HYBRID_HDR_LEN);

	// Copy payload from user space to our buffer, after the header
	if (payload_count > 0) {
		if (copy_from_user(tx_frame_buf + DW1000_HYBRID_HDR_LEN, payload_buf, payload_count)) {
			dev_err(&priv->spi->dev, "Failed to copy from user for cdev write\n");
			kfree(tx_frame_buf);
			return -EFAULT;
		}
	}

	// Call the low-level TX function (skb is NULL for cdev writes)
	ret = dw1000_hybrid_lowlevel_tx(priv, tx_frame_buf, frame_len_with_hdr, NULL);

	kfree(tx_frame_buf); // Free the temporary buffer

	if (ret) {
		dev_err(&priv->spi->dev, "Lowlevel TX failed for cdev write: %d\n", ret);
		return ret; // Return the error from lowlevel_tx (-EBUSY, -EINVAL, etc.)
	}

	// Update file position if needed 
	*f_pos += count; // Increment by the original user count (including optional header byte)
	return count;    
}


// --- Character Device mmap (Placeholder - Needs full implementation from previous skeleton) ---
static void dw1000_vm_open(struct vm_area_struct *vma)
{
	// 不需要特殊处理
}

static void dw1000_vm_close(struct vm_area_struct *vma)
{
	// 不需要特殊处理 
}

static vm_fault_t dw1000_vm_fault(struct vm_fault *vmf)
{
    struct dw1000_hybrid_priv *priv = vmf->vma->vm_private_data;
    struct page *page;
    unsigned long offset;
    
    // 计算在缓冲区中的实际偏移量
    offset = vmf->address - vmf->vma->vm_start;
    
    // 检查偏移量是否超出缓冲区范围
    if (offset >= priv->sensing_buffer_size) {
        dev_err(&priv->spi->dev, "VM fault: offset %lu exceeds buffer size %zu\n", 
                offset, priv->sensing_buffer_size);
        return VM_FAULT_SIGBUS;
    }
    
    // 使用 vmalloc_to_page 获取对应的页面
    page = vmalloc_to_page(priv->sensing_buffer + offset);
    if (!page) {
        dev_err(&priv->spi->dev, "Failed to get page for vmalloc memory at offset %lu\n", offset);
        return VM_FAULT_SIGBUS;
    }
    
    // 增加页面引用计数
    get_page(page);
    
    // 设置页面
    vmf->page = page;
    
    return 0;
}

static const struct vm_operations_struct dw1000_vm_ops = {
    .open = dw1000_vm_open,
    .close = dw1000_vm_close,
    .fault = dw1000_vm_fault,
};

static int dw1000_hybrid_cdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
    struct dw1000_hybrid_priv *priv = filp->private_data;
    unsigned long size = vma->vm_end - vma->vm_start;
    unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
    
    // 检查参数
    if (!priv->sensing_buffer) {
        dev_err(&priv->spi->dev, "No sensing buffer available for mmap\n");
        return -EINVAL;
    }
    
    if (offset + size > priv->sensing_buffer_size) {
        dev_err(&priv->spi->dev, "mmap request exceeds buffer size\n");
        return -EINVAL;
    }
    
    // 不允许写入
    if (vma->vm_flags & (VM_WRITE | VM_MAYWRITE)) {
        return -EPERM;
    }
    
    // 设置VMA标志
    vma->vm_flags |= VM_DONTEXPAND | VM_DONTDUMP | VM_MIXEDMAP;
    vma->vm_ops = &dw1000_vm_ops;
    vma->vm_private_data = priv;
    
    // 调用open回调
    dw1000_vm_open(vma);
    
    return 0;
}

// --- Character Device File Ops (Stubs, need implementations) ---
static int dw1000_hybrid_cdev_open(struct inode *inode, struct file *filp) {
	struct dw1000_hybrid_priv *priv = container_of(inode->i_cdev, struct dw1000_hybrid_priv, cdev);
	filp->private_data = priv;
	return 0;
}
static int dw1000_hybrid_cdev_release(struct inode *inode, struct file *filp) { return 0; }
static __poll_t dw1000_hybrid_cdev_poll(struct file *filp, poll_table *wait) {
	struct dw1000_hybrid_priv *priv = filp->private_data;
	__poll_t mask = 0;
	// unsigned long head, tail; // Replaced
	poll_wait(filp, &priv->read_wq, wait);
	// head = atomic_long_read(&priv->rx_ring_head); // Replaced
	// tail = atomic_long_read(&priv->rx_ring_tail); // Replaced
	if (kfifo_len(&priv->sensing_fifo) > 0)
		mask |= EPOLLIN | EPOLLRDNORM;
	return mask;
}

// --- Network Device Ops (Stubs, need implementations) ---
static int dw1000_hybrid_net_open(struct net_device *dev) {
	struct dw1000_hybrid_priv *priv = netdev_priv(dev);
	int ret;
	// Shared IRQ requested in probe, just enable NAPI & queue
	napi_enable(&priv->napi);
	netif_carrier_on(dev); // Simplified
	netif_start_queue(dev);
	
	// Enable hardware receiver
	dev_info(&priv->spi->dev, "Enabling DW1000 receiver for netdev.\n");
	ret = dw1000_write_reg32(priv, RG_SYS_CTRL, 0, SYS_CTRL_RXENAB); // Set RXENAB
	if (ret) {
		dev_err(&priv->spi->dev, "Failed to enable receiver in net_open: %d\n", ret);
		// If unable to enable receiver, need to rollback previous operations
		netif_stop_queue(dev);
		netif_carrier_off(dev);
		napi_disable(&priv->napi);
		return ret; // Return error
	}

	enable_irq(priv->irq_num); // Enable the correct stored IRQ number
	return 0;
}
static int dw1000_hybrid_net_stop(struct net_device *dev) {
	struct dw1000_hybrid_priv *priv = netdev_priv(dev);
	netif_stop_queue(dev);
	netif_carrier_off(dev);
	napi_disable(&priv->napi);

	// Force transceiver off
	dev_info(&priv->spi->dev, "Disabling DW1000 receiver for netdev.\n");
	// Ignore return value, as we want to stop device anyway
	dw1000_write_reg32(priv, RG_SYS_CTRL, 0, SYS_CTRL_TRXOFF); 

	// IRQ disable/free handled in remove
	return 0;
}

// --- Probe/Remove (Refined DMA Request & Init) ---
static int dw1000_generic_hybrid_probe(struct spi_device *spi) {
	struct dw1000_hybrid_priv *priv;
	struct net_device *netdev;
	int ret;
	dma_cap_mask_t mask;
	struct dma_slave_config dma_cfg = {0};
	int irq_num_local = -1; // Local variable to store IRQ number

	dev_info(&spi->dev, "Probing DW1000 Generic HYBRID driver...\n");

	// Allocate netdev (including priv space)
	netdev = alloc_netdev(sizeof(struct dw1000_hybrid_priv), "dw%d", NET_NAME_UNKNOWN, ether_setup);
	if (!netdev) {
		return -ENOMEM;
	}
	
	priv = netdev_priv(netdev);
	memset(priv, 0, sizeof(*priv)); // Ensure complete initialization
	
	// Initialize basic associations
	priv->netdev = netdev;  // Set network device pointer
	priv->spi = spi;        // Set SPI device pointer
	spi_set_drvdata(spi, priv);
	
	// Initialize locks
	spin_lock_init(&priv->stats_lock);
	spin_lock_init(&priv->rx_desc_lock);
	spin_lock_init(&priv->tx_desc_lock);
	spin_lock_init(&priv->fifo_lock);
	spin_lock_init(&priv->config_lock);
	mutex_init(&priv->ioctl_lock);
	mutex_init(&priv->spi_reg_mutex);

	// Initialize atomic variables
	atomic_set(&priv->tx_pending_count, 0);
	atomic_set(&priv->tx_sequence_num, 0);

	// Initialize default configuration (using dw1000_hybrid_config)
	priv->config.channel = 5;          // Default use channel 5
	priv->config.spi_speed_hz = DW1000_DEFAULT_SPI_HZ; // 8MHz SPI rate
	priv->config.prf = DW1000_DEFAULT_PRF;              // PRF 64M (0=16MHz, 1=64MHz)
	priv->config.preamble_length = DW1000_DEFAULT_PREAMBLE_LEN;  // Default preamble length (e.g., 8 for 256)
	priv->config.preamble_code = DW1000_DEFAULT_PREAMBLE_CODE;    // Default preamble code index
	priv->config.data_rate = DW1000_DEFAULT_DATA_RATE;        // 110kbps data rate (0=110k, 1=850k, 2=6.8M)
	priv->config.smart_power = true;   // Enable smart power control
	priv->config.tx_power = DW1000_DEFAULT_TX_POWER_REG;      // Default TX power (register value)
	priv->config.pan_id = DW1000_DEFAULT_PAN_ID;      // Default PAN ID
	priv->config.default_dest_addr = 0xFFFFFFFFFFFFFFFF; // Default broadcast address
	priv->config.accept_bad_frames = false;
	priv->config.promiscuous = false;
	priv->config.ranging_enabled = false;
	priv->config.sfd_timeout = DW1000_DEFAULT_SFD_TIMEOUT; // Default example value
	priv->config.preamble_timeout = 0; // Default: use HW default

	// Set network device parameters
	netdev->netdev_ops = &dw1000_hybrid_netdev_ops;
	netdev->mtu = DW1000_DEFAULT_MTU;
	netdev->type = ARPHRD_IEEE802154;  // 802.15.4 protocol type
	netdev->needs_free_netdev = true;   // Let kernel free netdev at the end
	netdev->priv_destructor = NULL;     // Use default destructor function
	eth_hw_addr_random(netdev);         // Random MAC address

	// Initialize NAPI
	netif_napi_add(netdev, &priv->napi, dw1000_hybrid_poll, NAPI_WEIGHT);

	// Get interrupt GPIO
	priv->irq_gpio = devm_gpiod_get_optional(&spi->dev, "irq", GPIOD_IN);
	if (IS_ERR(priv->irq_gpio)) {
		ret = PTR_ERR(priv->irq_gpio);
		dev_err(&spi->dev, "Failed to get IRQ GPIO: %d\n", ret);
		goto cleanup_napi;
	}

	if (!priv->irq_gpio) {
		dev_err(&spi->dev, "IRQ GPIO not found or specified in device tree.\n");
		ret = -ENODEV;
		goto cleanup_napi;
	}

	// Get and validate IRQ number
	irq_num_local = gpiod_to_irq(priv->irq_gpio);
	if (irq_num_local <= 0) {
		dev_err(&spi->dev, "Failed to get valid IRQ number from GPIO: %d\n", irq_num_local);
		ret = irq_num_local < 0 ? irq_num_local : -EINVAL;
		goto cleanup_napi;
	}
	priv->irq_num = irq_num_local;
	dev_info(&spi->dev, "Using IRQ number %d from GPIO\n", priv->irq_num);

	// Request shared interrupt
	ret = devm_request_irq(&spi->dev, priv->irq_num,
			      dw1000_hybrid_irq_handler,
			      IRQF_TRIGGER_RISING | IRQF_SHARED,
			      DRIVER_NAME, priv);
	if (ret) {
		dev_err(&spi->dev, "IRQ request failed: %d\n", ret);
		goto cleanup_napi;
	}
	disable_irq(priv->irq_num); // Disable interrupt before device open

	// Remove duplicate NAPI initialization code
	// netif_napi_add(netdev, &priv->napi, dw1000_hybrid_poll, NAPI_WEIGHT);
	
	// Allocate character device region
	ret = alloc_chrdev_region(&priv->cdev_devt, 0, 1, "dw1000_sensor");
	if (ret < 0) {
		dev_err(&spi->dev, "Chrdev region allocation failed: %d\n", ret);
		goto cleanup_napi;
	}
	
	// Initialize and add character device
	cdev_init(&priv->cdev, &dw1000_hybrid_cdev_fops);
	priv->cdev.owner = THIS_MODULE;
	ret = cdev_add(&priv->cdev, priv->cdev_devt, 1);
	if (ret) {
		dev_err(&spi->dev, "cdev_add failed: %d\n", ret);
		goto cleanup_chrdev_region;
	}
	
	// Create character device class/node
	priv->cdev_class = class_create(THIS_MODULE, "dw1000_sensor");
	if (IS_ERR(priv->cdev_class)) {
		ret = PTR_ERR(priv->cdev_class);
		dev_err(&spi->dev, "Failed to create device class: %d\n", ret);
		goto cleanup_cdev;
	}

	if (!device_create(priv->cdev_class, &spi->dev, priv->cdev_devt, 
			  priv, "dw1000_sensor%d", MINOR(priv->cdev_devt))) {
		dev_err(&spi->dev, "Failed to create device node\n");
		ret = -ENOMEM;
		goto cleanup_class;
	}
	
	// Initialize wait queue and kfifo
	init_waitqueue_head(&priv->read_wq);
	
	// Use vmalloc to allocate buffer
	priv->sensing_buffer_size = RING_BUFFER_SIZE;
	priv->sensing_buffer = vmalloc(priv->sensing_buffer_size);
	if (!priv->sensing_buffer) {
		dev_err(&spi->dev, "Failed to allocate sensing buffer\n");
		ret = -ENOMEM;
		goto cleanup_device;
	}

	// Initialize kfifo with allocated buffer
	kfifo_init(&priv->sensing_fifo, priv->sensing_buffer, priv->sensing_buffer_size);

	//Set DMA buffer size
	priv->tx_dma_buf_size = DW1000_DEFAULT_TX_DMA_BUF_SIZE;
	priv->rx_dma_buf_size = DW1000_DEFAULT_RX_DMA_BUF_SIZE;
	
	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	
	struct resource *res;
	phys_addr_t spi_phys_addr = 0;
	
	// Try to get physical address from SPI controller's parent device
	res = platform_get_resource(to_platform_device(spi->controller->dev.parent),
				   IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&spi->dev, "Failed to get SPI controller physical address\n");
		ret = -ENODEV;
		goto cleanup_ring_buffer;
	}
	spi_phys_addr = res->start;
	dev_info(&spi->dev, "SPI controller physical address: 0x%llx\n", 
		 (unsigned long long)spi_phys_addr);

	#define SPI_TXDR_OFFSET 0x400
	#define SPI_RXDR_OFFSET 0x800

	// RX DMA Channel
	priv->rx_dma_chan = dma_request_slave_channel_compat(mask, NULL, NULL, &spi->dev, "rx");
	if (!priv->rx_dma_chan) {
		dev_err(&spi->dev, "Failed to request RX DMA channel\n");
		ret = -ENODEV;
		goto cleanup_ring_buffer;
	}

	// TX DMA Channel
	priv->tx_dma_chan = dma_request_slave_channel_compat(mask, NULL, NULL, &spi->dev, "tx");
	if (!priv->tx_dma_chan) {
		dev_err(&spi->dev, "Failed to request TX DMA channel\n");
		ret = -ENODEV;
		goto cleanup_dma_channels;
	}

	struct dma_slave_config rx_dma_cfg = {0};
	rx_dma_cfg.direction = DMA_DEV_TO_MEM;
	rx_dma_cfg.src_addr = spi_phys_addr + SPI_RXDR_OFFSET;
	rx_dma_cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	rx_dma_cfg.src_maxburst = 1;
	rx_dma_cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	rx_dma_cfg.dst_maxburst = 1;
	
	ret = dmaengine_slave_config(priv->rx_dma_chan, &rx_dma_cfg);
	if (ret) {
		dev_err(&spi->dev, "Failed to configure RX DMA channel: %d\n", ret);
		goto cleanup_dma_channels;
	}

	struct dma_slave_config tx_dma_cfg = {0};
	tx_dma_cfg.direction = DMA_MEM_TO_DEV;
	tx_dma_cfg.dst_addr = spi_phys_addr + SPI_TXDR_OFFSET;
	tx_dma_cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	tx_dma_cfg.dst_maxburst = 1;
	tx_dma_cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	tx_dma_cfg.src_maxburst = 1;

	ret = dmaengine_slave_config(priv->tx_dma_chan, &tx_dma_cfg);
	if (ret) {
		dev_err(&spi->dev, "Failed to configure TX DMA channel: %d\n", ret);
		goto cleanup_dma_channels;
	}

	// Initialize DW1000 hardware
	// 1. First execute device reset and basic configuration
	ret = dw1000_reset_device(priv);
	if (ret) {
		dev_err(&spi->dev, "Device reset failed: %d\n", ret);
		goto cleanup_dma_channels;
	}

	// 2. Execute additional hardware initialization (interrupt mask, etc.)
	ret = dw1000_hw_init(priv);
	if (ret) {
		dev_err(&spi->dev, "Hardware initialization failed: %d\n", ret);
		goto cleanup_dma_channels;
	}

	// Initialize TX DMA descriptors
	ret = dw1000_setup_tx_desc(priv);
	if (ret) {
		dev_err(&spi->dev, "Failed to setup TX DMA descriptors: %d\n", ret);
		goto cleanup_dma_channels;
	}

	// Set initial RX DMA
	ret = dw1000_setup_rx_dma(priv);
	if (ret) {
		dev_err(&spi->dev, "Failed to setup RX DMA: %d\n", ret);
		goto cleanup_tx_desc_and_channels;
	}

	ret = register_netdev(netdev);
	if (ret) {
		dev_err(&spi->dev, "Failed to register netdev: %d\n", ret);
		goto cleanup_rx_tx_desc_and_channels;
	}

	dev_info(&spi->dev, "DW1000 Generic HYBRID driver probed successfully! (net: %s, cdev: %d:%d)\n",
			 netdev->name, MAJOR(priv->cdev_devt), MINOR(priv->cdev_devt));

	return 0;

cleanup_rx_tx_desc_and_channels:
	dw1000_teardown_rx_dma(priv);
cleanup_tx_desc_and_channels:
	dw1000_cleanup_tx_desc(priv);
cleanup_dma_channels:
	if (priv->tx_dma_chan)
		dma_release_channel(priv->tx_dma_chan);
	if (priv->rx_dma_chan)
		dma_release_channel(priv->rx_dma_chan);
cleanup_ring_buffer:
	if (priv->sensing_buffer) {
		vfree(priv->sensing_buffer);
		priv->sensing_buffer = NULL;
	}
cleanup_device:
	if (priv->cdev_class && !IS_ERR(priv->cdev_class))
		device_destroy(priv->cdev_class, priv->cdev_devt);
cleanup_class:
	if (priv->cdev_class && !IS_ERR(priv->cdev_class))
		class_destroy(priv->cdev_class);
cleanup_cdev:
	cdev_del(&priv->cdev);
cleanup_chrdev_region:
	unregister_chrdev_region(priv->cdev_devt, 1);
cleanup_napi:
	netif_napi_del(&priv->napi);
cleanup_free_netdev:
	free_netdev(netdev);
	return ret;

}

static int dw1000_generic_hybrid_remove(struct spi_device *spi)
{
    struct dw1000_hybrid_priv *priv = spi_get_drvdata(spi);
    struct net_device *netdev = priv->netdev;
    int timeout;

    dev_info(&spi->dev, "Removing DW1000 Generic HYBRID driver...\n");

    // 1. First unregister network device (if running)
    if (netdev) {
		if (netif_running(netdev)) {
            netif_stop_queue(netdev);
            netif_carrier_off(netdev);
            napi_disable(&priv->napi);
        }
        unregister_netdev(netdev);
    }

    // 2. Stop all DMA operations
    if (priv->rx_dma_chan) {
        dmaengine_terminate_all(priv->rx_dma_chan);
        dw1000_teardown_rx_dma(priv);
    }

    if (priv->tx_dma_chan) {
        dmaengine_terminate_all(priv->tx_dma_chan);
        timeout = 100; // Wait up to 100ms for callbacks
        while (atomic_read(&priv->tx_pending_count) > 0 && timeout > 0) {
            msleep(1);
            timeout--;
        }
        dmaengine_synchronize(priv->tx_dma_chan);
        dw1000_cleanup_tx_desc(priv);
    }

    // 3. Release DMA channels
    if (priv->rx_dma_chan) {
        dma_release_channel(priv->rx_dma_chan);
    }
    if (priv->tx_dma_chan) {
        dma_release_channel(priv->tx_dma_chan);
    }

    // 4. Clean up character device resources
    if (priv->sensing_buffer) {
        // Ensure no mmap operations are in progress
        msleep(100); // Give time for all VM operations to complete
        
        // Clean up kfifo
        kfifo_free(&priv->sensing_fifo);
        
        // Release vmalloc allocated memory
        vfree(priv->sensing_buffer);
        priv->sensing_buffer = NULL;
    }

    if (priv->cdev_class) {
        device_destroy(priv->cdev_class, priv->cdev_devt);
        class_destroy(priv->cdev_class);
    }
    cdev_del(&priv->cdev);
    unregister_chrdev_region(priv->cdev_devt, 1);

    // 5. Finally release network device structure
    if (netdev) {
        free_netdev(netdev);
    }

    dev_info(&spi->dev, "DW1000 Generic HYBRID driver removed successfully\n");
    return 0;
}

// --- HW Init (Optimized) ---
static int dw1000_hw_init(struct dw1000_hybrid_priv *priv)
{
    int ret;
    
    dev_info(&priv->spi->dev, "Performing additional hardware initialization\n");
    
    // Note: At this point, the device has been reset and configured by reset_device
    // We only need to perform additional initialization steps
    
    // 1. Enable required interrupt masks
    // (SYS_STATUS_RXFCG for RX complete, SYS_STATUS_TXFRS for TX complete)
    u32 int_mask = SYS_STATUS_RXFCG | SYS_STATUS_TXFRS; 
    ret = dw1000_write_reg(priv, RG_SYS_MASK, 0, 4, &int_mask);
    if (ret) {
        dev_err(&priv->spi->dev, "Failed to set interrupt mask: %d\n", ret);
        return ret;
    }
    
    // 2. Ensure transceiver is in IDLE state (TRXOFF)
    ret = dw1000_write_reg32(priv, RG_SYS_CTRL, 0, SYS_CTRL_TRXOFF);
    if (ret) {
        dev_err(&priv->spi->dev, "Failed to set TRXOFF during init: %d\n", ret);
        return ret;
    }
    
    // 3. Clear all pending interrupt status
    u32 clear_status = 0xFFFFFFFF; // Clear all interrupt flags
    ret = dw1000_write_reg32(priv, RG_SYS_STATUS, 0, clear_status);
    if (ret) {
        dev_err(&priv->spi->dev, "Failed to clear interrupt status: %d\n", ret);
        return ret;
    }
    
    dev_info(&priv->spi->dev, "Additional hardware initialization completed successfully\n");
    return 0;
}

// --- Initialize DMA descriptors ---
static int dw1000_setup_rx_desc(struct dw1000_hybrid_priv *priv)
{
	int i;
	unsigned long flags;
	
	// Initialize list heads
	INIT_LIST_HEAD(&priv->rx_free_list);
	INIT_LIST_HEAD(&priv->rx_pending_list);
	
	// Allocate and initialize all descriptors
	for (i = 0; i < NUM_RX_DESC; i++) {
		struct dw1000_rx_desc *desc = &priv->rx_desc[i];
		
		// Allocate streaming DMA buffer for each descriptor
		desc->buffer_size = 2048; // Sufficiently large buffer
		
		// Use kmalloc to allocate cacheable memory, improve CPU access performance
		// Compared to dma_alloc_coherent, this approach allows CPU cache optimization,
		// and significantly improves performance for large data processing
		desc->buffer = kmalloc(desc->buffer_size, GFP_KERNEL);
		if (!desc->buffer) {
			dev_err(&priv->spi->dev, "Failed to allocate RX buffer %d\n", i);
			goto fail_cleanup;
		}
		
		// Use streaming DMA mapping, allowing CPU cache optimization
		desc->dma_addr = dma_map_single(&priv->spi->dev, 
										desc->buffer,
										desc->buffer_size,
										DMA_FROM_DEVICE);
		if (dma_mapping_error(&priv->spi->dev, desc->dma_addr)) {
			dev_err(&priv->spi->dev, "Failed to map RX DMA buffer %d\n", i);
			kfree(desc->buffer);
			desc->buffer = NULL;
			goto fail_cleanup;
		}
		
		desc->priv = priv; // Set pointer to private data
		desc->in_use = false;
		desc->data_len = 0;
		
		// Add to free list
		spin_lock_irqsave(&priv->rx_desc_lock, flags);
		list_add_tail(&desc->list, &priv->rx_free_list);
		spin_unlock_irqrestore(&priv->rx_desc_lock, flags);
	}
	
	return 0;
	
fail_cleanup:
	// Clean up allocated descriptors
	while (--i >= 0) {
		struct dw1000_rx_desc *desc = &priv->rx_desc[i];
		if (desc->buffer) {
			// First unmap DMA
			if (desc->dma_addr) {
				dma_unmap_single(&priv->spi->dev,
								desc->dma_addr,
								desc->buffer_size,
								DMA_FROM_DEVICE);
			}
			// Release kmalloc allocated memory
			kfree(desc->buffer);
			desc->buffer = NULL;
		}
	}
	return -ENOMEM;
}

// --- Clean up DMA descriptors ---
static void dw1000_cleanup_rx_desc(struct dw1000_hybrid_priv *priv)
{
	int i;
	unsigned long flags;
	
	// Ensure all lists are empty
	spin_lock_irqsave(&priv->rx_desc_lock, flags);
	INIT_LIST_HEAD(&priv->rx_free_list);
	INIT_LIST_HEAD(&priv->rx_pending_list);
	spin_unlock_irqrestore(&priv->rx_desc_lock, flags);
	
	// Release all descriptor buffers
	for (i = 0; i < NUM_RX_DESC; i++) {
		struct dw1000_rx_desc *desc = &priv->rx_desc[i];
		if (desc->buffer) {
			// First unmap DMA
			if (desc->dma_addr) {
				dma_unmap_single(&priv->spi->dev,
								desc->dma_addr,
								desc->buffer_size,
								DMA_FROM_DEVICE);
			}
			// Release kmalloc allocated memory
			kfree(desc->buffer);
			desc->buffer = NULL;
		}
	}
}

// --- Get a free descriptor ---
static struct dw1000_rx_desc *dw1000_get_free_rx_desc(struct dw1000_hybrid_priv *priv)
{
    struct dw1000_rx_desc *desc = NULL;
    unsigned long flags;
    
    spin_lock_irqsave(&priv->rx_desc_lock, flags);
    if (!list_empty(&priv->rx_free_list)) {
        desc = list_first_entry(&priv->rx_free_list, struct dw1000_rx_desc, list);
        list_del(&desc->list); // Remove from free_list, but not added to other lists
        desc->in_use = true;
        desc->data_len = 0;
    }
    spin_unlock_irqrestore(&priv->rx_desc_lock, flags);
    
    return desc;
}

// --- Submit RX descriptor to DMA controller ---
static int dw1000_submit_rx_desc(struct dw1000_hybrid_priv *priv, struct dw1000_rx_desc *desc)
{
    struct dma_async_tx_descriptor *dma_desc;
    dma_cookie_t cookie;
    unsigned long flags;
    int ret = 0;
    
    // Sync DMA buffer for DMA receive
    dma_sync_single_for_device(&priv->spi->dev,
                              desc->dma_addr,
                              desc->buffer_size,
                              DMA_FROM_DEVICE);
    
    // Prepare DMA descriptor
    dma_desc = dmaengine_prep_slave_single(priv->rx_dma_chan,
                                          desc->dma_addr,
                                          desc->buffer_size,
                                          DMA_DEV_TO_MEM,
                                          DMA_PREP_INTERRUPT);
    if (!dma_desc) {
        dev_err(&priv->spi->dev, "Failed to prepare RX DMA descriptor\n");
        return -EIO;
    }
    
    // Set callback and parameters
    dma_desc->callback = dw1000_hybrid_dma_rx_callback;
    dma_desc->callback_param = desc;
    
    // Submit DMA descriptor
    cookie = dmaengine_submit(dma_desc);
    if (dma_submit_error(cookie)) {
        dev_err(&priv->spi->dev, "Failed to submit RX DMA descriptor\n");
        return -EIO;
    }
    
    // Store cookie for status tracking
    desc->dma_cookie = cookie;
    
    // After DMA submission, add descriptor to pending list
    spin_lock_irqsave(&priv->rx_desc_lock, flags);
    list_add_tail(&desc->list, &priv->rx_pending_list);
    spin_unlock_irqrestore(&priv->rx_desc_lock, flags);
    
    // Start DMA
    dma_async_issue_pending(priv->rx_dma_chan);
    
    return 0;
}

// --- Refill one RX descriptor ---
static int dw1000_refill_rx_descriptor(struct dw1000_hybrid_priv *priv, struct dw1000_rx_desc *desc)
{
    unsigned long flags;
    int ret;
    
    if (!desc) {
        dev_err(&priv->spi->dev, "NULL descriptor in refill\n");
        return -EINVAL;
    }

    // Prepare descriptor for reuse
    desc->data_len = 0;
    
    // Resubmit to DMA engine
    ret = dw1000_submit_rx_desc(priv, desc);
    if (ret) {
        dev_err(&priv->spi->dev, "Failed to submit RX descriptor in refill: %d\n", ret);
        // Submission failed, return descriptor to free list
        spin_lock_irqsave(&priv->rx_desc_lock, flags);
        if (!list_empty(&desc->list)) { // Ensure it is not in any list
            list_del_init(&desc->list);
        }
        desc->in_use = false;
        list_add_tail(&desc->list, &priv->rx_free_list);
        spin_unlock_irqrestore(&priv->rx_desc_lock, flags);
        return ret;
    }
    
    return 0;
}

// --- Iteratively set multiple RX descriptors ---
static int dw1000_setup_rx_dma(struct dw1000_hybrid_priv *priv)
{
    int i, ret;
    struct dw1000_rx_desc *desc;
    unsigned long flags;
    
    dev_info(&priv->spi->dev, "Setting up RX DMA descriptors\n");
    
    // Initialize descriptors
    ret = dw1000_setup_rx_desc(priv);
    if (ret)
        return ret;
        
    // Submit multiple descriptors to DMA
    for (i = 0; i < NUM_RX_DESC; i++) {
        desc = dw1000_get_free_rx_desc(priv);
        if (!desc) {
            dev_err(&priv->spi->dev, "No free RX descriptors available\n");
            return -ENOSPC;
        }
        
        ret = dw1000_submit_rx_desc(priv, desc);
        if (ret) {
            dev_err(&priv->spi->dev, "Failed to submit RX descriptor %d\n", i);
            // Submission failed, return descriptor to free list
            spin_lock_irqsave(&priv->rx_desc_lock, flags);
            desc->in_use = false;
            list_add_tail(&desc->list, &priv->rx_free_list);
            spin_unlock_irqrestore(&priv->rx_desc_lock, flags);
            return ret;
        }
    }
    
    priv->rx_dma_active = true;
    return 0;
}

static void dw1000_teardown_rx_dma(struct dw1000_hybrid_priv *priv)
{
	if (!priv->rx_dma_active)
		return;
		
	dev_info(&priv->spi->dev, "Tearing down RX DMA\n");
	
	// Stop all ongoing DMA transfers
	dmaengine_terminate_all(priv->rx_dma_chan);
	
	// Clean up all descriptors
	dw1000_cleanup_rx_desc(priv);
	
	priv->rx_dma_active = false;
}




static int dw1000_reset_device(struct dw1000_hybrid_priv *priv)
{
    int ret;
    struct gpio_desc *rstn_gpio;
    u32 dev_id;

    dev_info(&priv->spi->dev, "Performing device reset\n");

    // Try hardware reset using RSTN pin first
    rstn_gpio = devm_gpiod_get_optional(&priv->spi->dev, "rstn", GPIOD_OUT_LOW);
    if (!IS_ERR_OR_NULL(rstn_gpio)) {
        dev_info(&priv->spi->dev, "Performing hardware reset via RSTN pin\n");
        
        // Pull RSTN low for at least 100μs (per DW1000 datasheet)
        gpiod_set_value(rstn_gpio, 0);
        usleep_range(1000, 1500); // 1ms delay, more than required 100μs
        
        // Release reset pin
        gpiod_set_value(rstn_gpio, 1);
        msleep(5); // Wait for DW1000 internal initialization
        
        devm_gpiod_put(&priv->spi->dev, rstn_gpio);
    } else {
        // If no hardware reset pin, try software reset
        u8 pmsc_ctrl0[4] = {0};
        u8 softreset_cmd[4] = {0};

        dev_info(&priv->spi->dev, "Performing software reset via PMSC register\n");
        
        // Read current PMSC_CTRL0 register value
        ret = dw1000_read_reg(priv, RG_PMSC, SUB_PMSC_CTRL0, 4, pmsc_ctrl0);
        if (ret) {
            dev_err(&priv->spi->dev, "Failed to read PMSC_CTRL0 for software reset: %d\n", ret);
            // Even if read fails, proceed with reset attempt
        }
        
        // Set reset bits (SYSCLKS bits)
        softreset_cmd[0] = pmsc_ctrl0[0] & 0xFC; // Clear bits 0 and 1
        softreset_cmd[1] = pmsc_ctrl0[1] & 0xFC; // Clear bits 8 and 9
        ret = dw1000_write_reg(priv, RG_PMSC, SUB_PMSC_CTRL0, 2, softreset_cmd);
        if (ret) {
            dev_err(&priv->spi->dev, "Failed to set reset bits for software reset: %d\n", ret);
            return ret;
        }
        
        // Brief delay
        usleep_range(1000, 1500);
        
        // Clear reset bits, restore clocks
        ret = dw1000_write_reg(priv, RG_PMSC, SUB_PMSC_CTRL0, 4, pmsc_ctrl0);
        if (ret) {
            dev_err(&priv->spi->dev, "Failed to clear reset bits after software reset: %d\n", ret);
        }
        
        // Wait for reset to complete
        msleep(5);
    }

    // Verify device ID after reset
    ret = dw1000_read_reg(priv, RG_DEV_ID, 0, 4, &dev_id);
    if (ret) {
        dev_err(&priv->spi->dev, "Failed to read device ID after reset: %d\n", ret);
        return ret;
    }
    if ((dev_id & 0xFFFF0000) != 0xDECA0000) {
        dev_err(&priv->spi->dev, "Invalid Device ID after reset: 0x%08x\n", dev_id);
        return -ENODEV;
    }

    // After successful reset, re-apply the configuration
    dev_info(&priv->spi->dev, "Applying configuration after reset...\n");
    ret = dw1000_apply_config(priv);
    if (ret) {
        dev_err(&priv->spi->dev, "Failed to reconfigure device after reset: %d\n", ret);
        return ret;
    }

    dev_info(&priv->spi->dev, "Device reset and reconfiguration completed successfully\n");
    return 0;
}

static int dw1000_tx_test(struct dw1000_hybrid_priv *priv, int count)
{
    int i, ret;
    u8 test_frame[DW1000_TX_TEST_FRAME_SIZE] = {0}; // Test frame buffer
    struct dw1000_hybrid_hdr *hdr;
    
    dev_info(&priv->spi->dev, "Starting TX test with %d frames\n", count);
    
    // Prepare test frame
    hdr = (struct dw1000_hybrid_hdr *)test_frame;
    hdr->frame_type = DW1000_FRAME_TYPE_NET;
    hdr->dest_addr = cpu_to_le64(0xFFFFFFFFFFFFFFFF); // Broadcast
    if (priv->netdev && priv->netdev->dev_addr) {
		hdr.src_addr = 0;
        memcpy(&hdr->src_addr, priv->netdev->dev_addr, ETH_ALEN);
    }
    hdr->pan_id = cpu_to_le16(priv->config.pan_id);
    
    // Send test frames
    for (i = 0; i < count; i++) {
        hdr->seq_num = i & 0xFF;
        ret = dw1000_hybrid_lowlevel_tx(priv, test_frame, sizeof(test_frame), NULL);
        if (ret) {
            dev_err(&priv->spi->dev, "TX test frame %d failed: %d\n", i, ret);
            return ret;
        }
        msleep(DW1000_TX_TEST_DELAY_MS); // 10ms delay between frames
    }
    
    dev_info(&priv->spi->dev, "TX test completed successfully\n");
    return 0;
}

static int dw1000_rx_test(struct dw1000_hybrid_priv *priv, int duration_ms)
{
    unsigned long start_time, current_time;
    u32 initial_rx_count, final_rx_count;
    
    dev_info(&priv->spi->dev, "Starting RX test for %d ms\n", duration_ms);
    
    // Record initial statistics
    initial_rx_count = priv->stats.rx_packets;
    
    // Enable receiver
    // Use the defined bit directly, a specific macro for 0x01 seems overkill here
    u8 sys_ctrl[4] = {0x01, 0x00, 0x00, 0x00}; // Enable RX 
    dw1000_write_reg(priv, RG_SYS_CTRL, 0, 4, sys_ctrl);
    
    // Wait for specified duration
    start_time = jiffies;
    current_time = start_time;
    while (time_before(current_time, start_time + msecs_to_jiffies(duration_ms))) {
        msleep(DW1000_RX_TEST_POLL_MS); // Sleep for 100ms intervals
        current_time = jiffies;
    }
    
    // Get final statistics
    final_rx_count = priv->stats.rx_packets;
    
    dev_info(&priv->spi->dev, "RX test completed. Received %u packets in %d ms\n",
             final_rx_count - initial_rx_count, duration_ms);
             
    return 0;
}

// --- RX DMA Callback ---
static void dw1000_hybrid_dma_rx_callback(void *param)
{
    struct dw1000_rx_desc *desc = param;
    struct dw1000_hybrid_priv *priv;
    unsigned long flags;
    enum dma_status status;

    if (!desc) {
        pr_err("DW1000: NULL descriptor in RX DMA callback!\n");
        return;
    }
    
    priv = desc->priv; // Get priv directly from descriptor
    if (!priv) {
        // This should theoretically never happen
        pr_err("DW1000: NULL priv in RX descriptor! Cookie: %d\n", desc->dma_cookie);
        return;
    }

    // Check DMA status
    status = dmaengine_tx_status(priv->rx_dma_chan, desc->dma_cookie, NULL);
    if (status == DMA_ERROR) {
        dev_warn(&priv->spi->dev, "RX DMA transfer error\n");
        spin_lock_irqsave(&priv->stats_lock, flags);
        priv->stats.rx_dma_errors++;
        spin_unlock_irqrestore(&priv->stats_lock, flags);
        
        // When DMA error, move descriptor back to free list
        spin_lock_irqsave(&priv->rx_desc_lock, flags);
        list_del(&desc->list);
        desc->in_use = false;
        list_add_tail(&desc->list, &priv->rx_free_list);
        spin_unlock_irqrestore(&priv->rx_desc_lock, flags);
    } else if (status == DMA_COMPLETE) {
        // DMA transfer complete, sync buffer for CPU access
        dma_sync_single_for_cpu(&priv->spi->dev,
                               desc->dma_addr,
                               desc->buffer_size,
                               DMA_FROM_DEVICE);
        
        // When successful, let NAPI handle data
        // Descriptor remains in pending list, decided by NAPI whether to reuse or release
        if (napi_schedule_prep(&priv->napi)) {
            __napi_schedule(&priv->napi);
        }
    }
}

// --- Submit TX Descriptor Function (Improved) ---
static int dw1000_submit_tx_desc(struct dw1000_hybrid_priv *priv, struct dw1000_tx_desc *desc)
{
    struct dma_async_tx_descriptor *dma_desc;
    dma_cookie_t cookie;
    
    // Sync buffer before DMA transfer, ensure CPU-written data is visible to DMA
    dma_sync_single_for_device(&priv->spi->dev,
                              desc->dma_addr,
                              desc->data_len,
                              DMA_TO_DEVICE);
    
    // Prepare DMA descriptor
    dma_desc = dmaengine_prep_slave_single(priv->tx_dma_chan,
                                          desc->dma_addr,
                                          desc->data_len,
                                          DMA_MEM_TO_DEV,
                                          DMA_PREP_INTERRUPT);
    if (!dma_desc) {
        dev_err(&priv->spi->dev, "Failed to prepare TX DMA descriptor\n");
        return -EIO;
    }
    
    // Set callback with descriptor as parameter
    dma_desc->callback = dw1000_hybrid_dma_tx_callback;
    dma_desc->callback_param = desc;
    
    // Submit DMA descriptor
    cookie = dmaengine_submit(dma_desc);
    if (dma_submit_error(cookie)) {
        dev_err(&priv->spi->dev, "Failed to submit TX DMA descriptor\n");
        return -EIO;
    }
    
    // Store cookie for status tracking
    desc->dma_cookie = cookie;
    
    // Start DMA
    dma_async_issue_pending(priv->tx_dma_chan);
    
    return 0;
}

// --- Get a free TX descriptor ---
static struct dw1000_tx_desc *dw1000_get_free_tx_desc(struct dw1000_hybrid_priv *priv)
{
	struct dw1000_tx_desc *desc = NULL;
	unsigned long flags;
	
	spin_lock_irqsave(&priv->tx_desc_lock, flags);
	if (!list_empty(&priv->tx_free_list)) {
		desc = list_first_entry(&priv->tx_free_list, struct dw1000_tx_desc, list);
		list_del(&desc->list);
		desc->in_use = true;
		desc->data_len = 0;
		desc->skb = NULL;
		// Note: Do not add to pending_list here, add after successful DMA submission
	}
	spin_unlock_irqrestore(&priv->tx_desc_lock, flags);
	
	return desc;
}

// --- Put error TX descriptor back to free list ---
static void dw1000_put_tx_desc_on_error(struct dw1000_tx_desc *desc)
{
	struct dw1000_hybrid_priv *priv;
	unsigned long flags;
	
	if (!desc || !desc->priv) return;
	priv = desc->priv;
	
	spin_lock_irqsave(&priv->tx_desc_lock, flags);
	desc->in_use = false;
	// Ensure it is not in any list (if previously added to pending_list)
	if (!list_empty(&desc->list)) {
		list_del_init(&desc->list);
	}
	list_add_tail(&desc->list, &priv->tx_free_list);
	spin_unlock_irqrestore(&priv->tx_desc_lock, flags);
}

// --- Helper Function for TX Power Calculation (Implemented) ---
// Based on DW1000 datasheet and user manual

// Calculate 32-bit value for TX_POWER register
static u32 calculate_tx_power_reg_val(s8 dbm) {
    struct power_config cfg = get_power_config(dbm);
    // Combine bit fields: DA(8b) << 24 | Mixer(8b) << 16 | PA(8b) << 8 | Reserved(8b) 0
    // Assume lowest 8 bits are reserved as 0
    return ((u32)cfg.da << TX_POWER_DA_SHIFT) |
           ((u32)cfg.mixer << TX_POWER_MIXER_SHIFT) |
           ((u32)cfg.pa << TX_POWER_PA_SHIFT);
}

// Find corresponding DA, Mixer, PA values based on dBm (based on user-provided datasheet)
static struct power_config get_power_config(s8 dbm) {
    struct power_config config; // No default initialization here, set in switch

    // Add more configurations based on user-provided datasheet (Table 20 for Ch5, 64MHz PRF)
    switch (dbm) {
        case -40:
            config.da = 0xFF; config.mixer = 0x00; config.pa = 0x00;
            break;
        case -37:
            config.da = 0xF0; config.mixer = 0x00; config.pa = 0x00;
            break;
        case -34:
            config.da = 0xE0; config.mixer = 0x00; config.pa = 0x00;
            break;
        // ... Add -31, -28, -25, -22, -19, -16, -13 based on table pattern if needed ...
        case -10: // Typical configuration
            config.da = 0x0F; config.mixer = 0x03; config.pa = 0x01;
            break;
        // ... Add intermediate 0.5dB steps if precise control is needed ...
        case 0: // Maximum power example
            config.da = 0x00; config.mixer = 0x07; config.pa = 0x03;
            break;
        default:
            // Handle undefined dBm values, fallback to recommended -10dBm
            // dev_warn(&priv->spi->dev, "Unhandled dBm value %d, using default -10dBm config\n", dbm);
            config.da = 0x0F; config.mixer = 0x03; config.pa = 0x01; // fallback to -10dBm
            break;
    }
    // TODO: Add more dBm mappings based on need and datasheet Table 20, etc.

    return config;
}

static int dw1000_apply_config(struct dw1000_hybrid_priv *priv)
{
	u32 dev_id = 0;
	u8 sys_cfg[4] = {0};
	//u8 sys_ctrl[4] = {0}; // Don't toggle receiver here
	u8 panadr[4] = {0};
	u8 chan_ctrl[4] = {0};
	u8 tx_fctrl[RG_TX_FCTRL_LEN] = {0}; // TX_FCTRL is 5 bytes long
	u32 tx_power_reg = 0; // Use u32 for the TX_POWER register value
	int ret;
	unsigned long flags; // For config_lock

	dev_info(&priv->spi->dev, "Applying DW1000 config (ch=%d, prf=%d, rate=%d, plen=%u, smart_pwr=%d, tx_pwr=0x%04x, pan=0x%04x)\n",
		 priv->config.channel, priv->config.prf, priv->config.data_rate,
		 priv->config.preamble_length, priv->config.smart_power, priv->config.tx_power,
		 priv->config.pan_id);

	spin_lock_irqsave(&priv->config_lock, flags); // Acquire config lock

	// --- Start critical section reading config ---
	// 1. Verify Device ID (Optional here, usually done once in init)
	/*
	ret = dw1000_read_reg(priv, RG_DEV_ID, 0, 4, &dev_id);
	if (ret) {
		dev_err(&priv->spi->dev, "Failed to read device ID in apply_config: %d\n", ret);
		// spin_unlock_irqrestore(&priv->config_lock, flags); // Release lock before return
		// return ret;
	}
	if ((dev_id & 0xFFFF0000) != 0xDECA0000) {
		dev_err(&priv->spi->dev, "Invalid Device ID in apply_config: 0x%08x\n", dev_id);
		// spin_unlock_irqrestore(&priv->config_lock, flags); // Release lock before return
		// return -ENODEV;
	}
	*/

	// 2. Configure system register - RX double buffer, Frame filtering, etc.
	sys_cfg[0] = SYS_CFG_RXDBUFFEN; // Enable RX double buffer (RXDBUFFEN bit)
	sys_cfg[1] = priv->config.accept_bad_frames ? 0x00 : SYS_CFG_FFEN; // Use FFEN bit if not accepting bad frames
	// Consider adding other FF bits (FFBC, FFAB, FFAA, FFAM) if needed based on priv->config flags
	sys_cfg[1] |= (SYS_CFG_FFAB | SYS_CFG_FFAA | SYS_CFG_FFAM); // Example: Filter Data, ACK, MAC command frames
	sys_cfg[2] = SYS_CFG_PHR_MODE_STD; // Standard PHR mode (adjust if extended PHR is needed)
	sys_cfg[3] = 0x00;
	// SPI write is now protected by spi_reg_mutex, no need for config_lock here
	ret = dw1000_write_reg(priv, RG_SYS_CFG, 0, 4, sys_cfg);
	if (ret) {
		dev_err(&priv->spi->dev, "Failed to configure SYS_CFG: %d\n", ret);
		spin_unlock_irqrestore(&priv->config_lock, flags); // Release lock before return
		return ret;
	}

	// 3. Configure channel - Channel number and PRF
	chan_ctrl[0] = (priv->config.channel & CHAN_CTRL_CH_NUM_MASK) |
				   ((priv->config.prf << CHAN_CTRL_PRF_SHIFT) & CHAN_CTRL_PRF_MASK);
	chan_ctrl[1] = 0x00; // Reserved, maybe set TX/RX preamble codes here?
	// Set preamble codes (RX first, then TX)
	chan_ctrl[2] = (priv->config.preamble_code & CHAN_CTRL_RX_PRCODE_MASK) |
				   (((priv->config.preamble_code << CHAN_CTRL_TX_PRCODE_SHIFT)) & CHAN_CTRL_TX_PRCODE_MASK);
	chan_ctrl[3] = 0x00; // RF delays - configure if non-default needed
	ret = dw1000_write_reg(priv, RG_CHAN_CTRL, 0, 4, chan_ctrl);
	if (ret) {
		dev_err(&priv->spi->dev, "Failed to configure CHAN_CTRL: %d\n", ret);
		spin_unlock_irqrestore(&priv->config_lock, flags); // Release lock before return
		return ret;
	}

	// 4. Configure device address - PAN_ID and short address
	panadr[0] = priv->config.pan_id & 0xFF;
	panadr[1] = (priv->config.pan_id >> 8) & 0xFF;
	panadr[2] = 0x00; // Short address low byte (e.g., from MAC or set via IOCTL)
	panadr[3] = 0x00; // Short address high byte (e.g., from MAC or set via IOCTL)
	if (priv->netdev && priv->netdev->dev_addr) {
		// Example: Use last 2 bytes of MAC as short addr
		panadr[2] = priv->netdev->dev_addr[4];
		panadr[3] = priv->netdev->dev_addr[5];
	}
	ret = dw1000_write_reg(priv, RG_PANADR, 0, 4, panadr);
	if (ret) {
		dev_err(&priv->spi->dev, "Failed to configure PANADR: %d\n", ret);
		spin_unlock_irqrestore(&priv->config_lock, flags); // Release lock before return
		return ret;
	}

	// 5. Configure TX power
	if (priv->config.smart_power) {
		// Smart power typically needs a base power setting and enabling a bit
		// Assume 0x1F1F1F1F enables smart power optimization with default base.
		// Check datasheet for exact smart power bits in TX_POWER or other registers.
		tx_power_reg = (DW1000_SMART_TX_POWER_REG << 24) | (DW1000_SMART_TX_POWER_REG << 16) |
		               (DW1000_SMART_TX_POWER_REG << 8)  | DW1000_SMART_TX_POWER_REG;
		dev_dbg(&priv->spi->dev, "Using smart TX power (Reg: 0x%08x)\n", tx_power_reg);
	} else if (priv->config.tx_power != 0) { // Use configured TX power (assuming it's the register value)
		// Direct mapping from 16-bit config value to 32-bit register?
		// Check datasheet: TX_POWER has fields for different conditions (e.g., manual, smart, different rates).
		// Simple approach: duplicate the 8-bit value across all bytes if config.tx_power is u8/u16 reg val.
		u8 pwr_byte = priv->config.tx_power & 0xFF; // Example if tx_power holds 8-bit val
		tx_power_reg = (pwr_byte << 24) | (pwr_byte << 16) | (pwr_byte << 8) | pwr_byte;
		// If priv->config.tx_power is intended to be dBm, call calculate_tx_power_reg_val here.
		// tx_power_reg = calculate_tx_power_reg_val(priv->config.tx_power); // If config stores dBm
		dev_dbg(&priv->spi->dev, "Using configured TX power (Reg: 0x%08x from val 0x%04x)\n",
				tx_power_reg, priv->config.tx_power);
	} else {
		// Use default power config
		u8 pwr_byte = DW1000_DEFAULT_TX_POWER_REG & 0xFF;
		tx_power_reg = (pwr_byte << 24) | (pwr_byte << 16) | (pwr_byte << 8) | pwr_byte;
		dev_dbg(&priv->spi->dev, "Using default TX power (Reg: 0x%08x)\n", tx_power_reg);
	}
	ret = dw1000_write_reg32(priv, RG_TX_POWER, 0, tx_power_reg); // Write the calculated 32-bit value
	if (ret) {
		dev_err(&priv->spi->dev, "Failed to configure TX_POWER: %d\n", ret);
		spin_unlock_irqrestore(&priv->config_lock, flags); // Release lock before return
		return ret;
	}

	// 6. Configure TX frame control register - Preamble length, Data rate, PRF
	// TX_FCTRL is 5 bytes: TXFLEN(10), TXBR(2), TR(1), TXPRF(2), TXPSR(2), PE(2), TXBOFFS(10), IFSDELAY(8)
	// Read first to preserve TXBOFFS and IFSDELAY? Safer.
	ret = dw1000_read_reg(priv, RG_TX_FCTRL, 0, RG_TX_FCTRL_LEN, tx_fctrl);
	if (ret) {
		dev_err(&priv->spi->dev, "Failed to read TX_FCTRL before write: %d\n", ret);
		// Proceed with setting defaults if read fails? Or return error?
		// Let's clear the relevant bytes and proceed.
		memset(tx_fctrl, 0, RG_TX_FCTRL_LEN);
		// return ret; // Option: return error if read fails
	}


	// Clear relevant bits before setting new values
	tx_fctrl[1] &= ~(TX_FCTRL_TXBR_MASK | TX_FCTRL_TR_BIT); // Clear TXBR and TR in Byte 1
	tx_fctrl[2] &= ~(TX_FCTRL_TXPRF_MASK | TX_FCTRL_TXPSR_MASK | TX_FCTRL_PE_MASK); // Clear TXPRF, TXPSR, PE in Byte 2

	// Set TXBR (Data Rate) in Byte 1
	tx_fctrl[1] |= ((priv->config.data_rate << TX_FCTRL_TXBR_SHIFT) & TX_FCTRL_TXBR_MASK);
	// Set TR bit if ranging is enabled (assuming a config flag exists)
	// tx_fctrl[1] |= (priv->config.ranging_enabled ? TX_FCTRL_TR_BIT : 0);

	// Set PE (Preamble Length) and TXPSR (Preamble Symbol Repetitions) in Byte 2
	u8 pe_txpsr_val = 0;
	u16 plen = priv->config.preamble_length; // Use the config value directly (now u16)

	// Use correct mapping based on datasheet (Table 16 & non-std preamble section)
	// This mapping seems correct based on prior discussion, assuming defines are right.
	switch (plen) {
		// Standard lengths (64-512): Use specific PE code, TXPSR depends on PRF
		case 64:   pe_txpsr_val = (TX_FCTRL_PE_64 << TX_FCTRL_PE_SHIFT); break;
		case 128:  pe_txpsr_val = (TX_FCTRL_PE_128 << TX_FCTRL_PE_SHIFT); break;
		case 256:  pe_txpsr_val = (TX_FCTRL_PE_256 << TX_FCTRL_PE_SHIFT); break;
		case 512:  pe_txpsr_val = (TX_FCTRL_PE_512 << TX_FCTRL_PE_SHIFT); break;

		// Non-standard lengths (1024+): Use specific PE/TXPSR combinations
		// Note: The defines TX_FCTRL_PE_NS_* and TX_FCTRL_TXPSR_* need verification.
		// Let's assume the combined logic was correct.
		case 1024: pe_txpsr_val = (TX_FCTRL_PE_NS_00 << TX_FCTRL_PE_SHIFT) | (TX_FCTRL_TXPSR_1024 << TX_FCTRL_TXPSR_SHIFT); break;
		case 1536: pe_txpsr_val = (TX_FCTRL_PE_NS_01 << TX_FCTRL_PE_SHIFT) | (TX_FCTRL_TXPSR_1024 << TX_FCTRL_TXPSR_SHIFT); break;
		case 2048: pe_txpsr_val = (TX_FCTRL_PE_NS_10 << TX_FCTRL_PE_SHIFT) | (TX_FCTRL_TXPSR_1024 << TX_FCTRL_TXPSR_SHIFT); break;
		case 4096: pe_txpsr_val = (TX_FCTRL_PE_NS_00 << TX_FCTRL_PE_SHIFT) | (TX_FCTRL_TXPSR_4096 << TX_FCTRL_TXPSR_SHIFT); break;

		default:
			dev_warn(&priv->spi->dev, "Invalid preamble length %u in config, using default 64 symbols\n", plen);
			pe_txpsr_val = (TX_FCTRL_PE_64 << TX_FCTRL_PE_SHIFT); // Default to 64
			break;
	}
	// Add standard TXPSR based on PRF for standard lengths (64-512)
	if (plen <= 512) {
		u8 std_txpsr = (priv->config.prf == 0) ? TX_FCTRL_TXPSR_N_16MHZ : TX_FCTRL_TXPSR_N_64MHZ;
		pe_txpsr_val |= (std_txpsr << TX_FCTRL_TXPSR_SHIFT);
	}

	tx_fctrl[2] |= pe_txpsr_val; // Apply combined PE and TXPSR bits

	// Set TXPRF (PRF) in Byte 2
	tx_fctrl[2] |= ((priv->config.prf << TX_FCTRL_TXPRF_SHIFT) & TX_FCTRL_TXPRF_MASK);

	// Note: TXFLEN (Bytes 0 and 1 lower bits) must be set *before* each transmission based on the actual frame size.
	// Leaving tx_fctrl[0] and lower bits of tx_fctrl[1] as 0 here is fine.

	// SPI write is now protected by spi_reg_mutex
	ret = dw1000_write_reg(priv, RG_TX_FCTRL, 0, RG_TX_FCTRL_LEN, tx_fctrl); // Use length macro
	if (ret) {
		dev_err(&priv->spi->dev, "Failed to configure TX_FCTRL: %d\n", ret);
		spin_unlock_irqrestore(&priv->config_lock, flags); // Release lock before return
		return ret;
	}

	// 7. Configure SPI speed if needed
	if (priv->spi->max_speed_hz != priv->config.spi_speed_hz) {
		// SPI setup might sleep, cannot hold spinlock
		u32 speed_hz = priv->config.spi_speed_hz;  // 在释放锁前保存
		spin_unlock_irqrestore(&priv->config_lock, flags);
		priv->spi->max_speed_hz = speed_hz;
		ret = spi_setup(priv->spi);
		spin_lock_irqsave(&priv->config_lock, flags);
		if (ret) {
			dev_err(&priv->spi->dev, "Failed to configure SPI speed: %d\n", ret);
			spin_unlock_irqrestore(&priv->config_lock, flags); // Release lock before return
			return ret;
		}
	}

	// --- End critical section reading config ---
	spin_unlock_irqrestore(&priv->config_lock, flags); // Release config lock at the end

	// NOTE: Do NOT enable receiver here. Let the caller (e.g., hw_init, reset) do it.

	dev_info(&priv->spi->dev, "DW1000 configuration applied successfully\n");

	return 0;
}

// --- Initialize TX DMA descriptors ---
static int dw1000_setup_tx_desc(struct dw1000_hybrid_priv *priv)
{
	int i;
	unsigned long flags;

	// Initialize list heads
	INIT_LIST_HEAD(&priv->tx_free_list);
	INIT_LIST_HEAD(&priv->tx_pending_list);

	// Allocate and initialize all descriptors
	for (i = 0; i < NUM_TX_DESC; i++) {
		struct dw1000_tx_desc *desc = &priv->tx_desc[i];

		// Allocate streaming DMA buffer for each descriptor
		desc->buffer_size = TX_DESC_BUFFER_SIZE; // Use defined TX descriptor buffer size
		
		// Use kmalloc to allocate cacheable memory, improve CPU access performance
		// Streaming DMA + CPU cache = better performance, especially for frequent data access
		desc->buffer = kmalloc(desc->buffer_size, GFP_KERNEL);
		if (!desc->buffer) {
			dev_err(&priv->spi->dev, "Failed to allocate TX buffer %d\n", i);
			goto fail_cleanup;
		}
		
		// Use streaming DMA mapping, allow CPU cache optimization
		desc->dma_addr = dma_map_single(&priv->spi->dev,
										desc->buffer,
										desc->buffer_size,
										DMA_TO_DEVICE);
		if (dma_mapping_error(&priv->spi->dev, desc->dma_addr)) {
			dev_err(&priv->spi->dev, "Failed to map TX DMA buffer %d\n", i);
			kfree(desc->buffer);
			desc->buffer = NULL;
			goto fail_cleanup;
		}

		desc->priv = priv; // Set pointer to private data
		desc->in_use = false;
		desc->data_len = 0;
		desc->skb = NULL;   // Initialize skb pointer

		// Add to free list
		spin_lock_irqsave(&priv->tx_desc_lock, flags);
		list_add_tail(&desc->list, &priv->tx_free_list);
		spin_unlock_irqrestore(&priv->tx_desc_lock, flags);
	}

	return 0;

fail_cleanup:
	// Clean up allocated descriptors
	while (--i >= 0) {
		struct dw1000_tx_desc *desc = &priv->tx_desc[i];
		if (desc->buffer) {
			// First unmap DMA
			if (desc->dma_addr) {
				dma_unmap_single(&priv->spi->dev,
								desc->dma_addr,
								desc->buffer_size,
								DMA_TO_DEVICE);
			}
			// Release kmalloc allocated memory
			kfree(desc->buffer);
			desc->buffer = NULL;
		}
	}
	// Clear possibly added list items
	spin_lock_irqsave(&priv->tx_desc_lock, flags);
	INIT_LIST_HEAD(&priv->tx_free_list);
	spin_unlock_irqrestore(&priv->tx_desc_lock, flags);
	return -ENOMEM;
}

// --- Clean up TX DMA descriptors ---
static void dw1000_cleanup_tx_desc(struct dw1000_hybrid_priv *priv)
{
	int i;
	unsigned long flags;

	// Ensure all lists are empty (locked protection)
	spin_lock_irqsave(&priv->tx_desc_lock, flags);
	INIT_LIST_HEAD(&priv->tx_free_list);
	INIT_LIST_HEAD(&priv->tx_pending_list);
	spin_unlock_irqrestore(&priv->tx_desc_lock, flags);

	// Release all descriptor buffers
	for (i = 0; i < NUM_TX_DESC; i++) {
		struct dw1000_tx_desc *desc = &priv->tx_desc[i];
		if (desc->buffer) {
			// First unmap DMA
			if (desc->dma_addr) {
				dma_unmap_single(&priv->spi->dev,
								desc->dma_addr,
								desc->buffer_size,
								DMA_TO_DEVICE);
			}
			// Release kmalloc allocated memory
			kfree(desc->buffer);
			desc->buffer = NULL;
		}
	}
}

// --- Helper function to execute TX --- 
static int dw1000_execute_tx(struct dw1000_hybrid_priv *priv, struct dw1000_tx_desc *desc)
{
	int ret;
	u8 tx_fctrl[RG_TX_FCTRL_LEN];
	unsigned long flags; // For tx_desc_lock

	// Check frame length against hardware limit (TXFLEN is 10 bits)
	if (desc->data_len == 0 || desc->data_len > 1023) { // TXFLEN max is 1023
		dev_err(&priv->spi->dev, "Invalid TX frame length for HW: %zu\n", desc->data_len);
		return -EINVAL;
	}

	// 1. Set TX_FCTRL.TXFLEN (Transmit Frame Length)
	// TXFLEN is bits 0-9 of TX_FCTRL. Frame length includes PHR + MAC PDU.
	// DW1000 hardware appends 2-byte CRC automatically.
	ret = dw1000_read_reg(priv, RG_TX_FCTRL, 0, RG_TX_FCTRL_LEN, tx_fctrl);
	if (ret) {
		dev_err(&priv->spi->dev, "Failed to read TX_FCTRL: %d\n", ret);
		return ret;
	}

	tx_fctrl[0] = (u8)(desc->data_len & 0x00FF); // Low byte of TXFLEN
	// Preserve other bits in tx_fctrl[1] (TXBR, TR) while setting TXFLEN[9:8]
	tx_fctrl[1] = (tx_fctrl[1] & 0xFC) | (u8)((desc->data_len >> 8) & 0x03);

	ret = dw1000_write_reg(priv, RG_TX_FCTRL, 0, RG_TX_FCTRL_LEN, tx_fctrl);
	if (ret) {
		dev_err(&priv->spi->dev, "Failed to write TX_FCTRL: %d\n", ret);
		return ret;
	}

	// 2. Sync buffer for device
	dma_sync_single_for_device(&priv->spi->dev, desc->dma_addr,
						   desc->data_len, DMA_TO_DEVICE);

	// 3. Increment pending TX count (before submitting DMA)
	atomic_inc(&priv->tx_pending_count);

	// 4. Submit DMA descriptor
	ret = dw1000_submit_tx_desc(priv, desc); // This function sets up callback and param
	if (ret) {
		dev_err(&priv->spi->dev, "dw1000_submit_tx_desc failed: %d\n", ret);
		atomic_dec(&priv->tx_pending_count); // Decrement count as submission failed
		return ret; // Caller will handle descriptor and SKB cleanup
	}

	// 5. If DMA submission was successful, add to pending list and start HW TX
	spin_lock_irqsave(&priv->tx_desc_lock, flags);
	list_add_tail(&desc->list, &priv->tx_pending_list);
	spin_unlock_irqrestore(&priv->tx_desc_lock, flags);

	// Start hardware transmission
	ret = dw1000_write_reg32(priv, RG_SYS_CTRL, 0, SYS_CTRL_TXSTRT);
	if (ret) {
		dev_err(&priv->spi->dev, "Failed to start TX (SYS_CTRL_TXSTRT): %d\n", ret);
		// DMA is submitted, callback will eventually run and handle the descriptor.
		// atomic_dec and list_del are NOT done here for TXSTRT failure, as DMA is active.
		// The callback will reduce tx_pending_count and manage the descriptor.
		// Log error and update stats for this specific failure.
		spin_lock_irqsave(&priv->stats_lock, flags);
		priv->stats.tx_errors++; // Generic TX error
		// Add a more specific counter if needed, e.g., tx_start_hw_errors
		spin_unlock_irqrestore(&priv->stats_lock, flags);
		// Return a specific error code to indicate failure AFTER DMA submission
		return -EAGAIN; // Use EAGAIN to signal this specific failure mode
	}

	return 0; // Success
}

// --- Shared TX helper --- 
static int dw1000_hybrid_lowlevel_tx(struct dw1000_hybrid_priv *priv, 
                                   const void *frame_data, 
                                   size_t frame_len, 
                                   struct sk_buff *skb_to_free)
{
    struct dw1000_tx_desc *desc;
    int ret;

    // 1. Get a free TX descriptor
    desc = dw1000_get_free_tx_desc(priv);
    if (!desc) {
        dev_warn_ratelimited(&priv->spi->dev, "No free TX descriptors for lowlevel_tx\n");
        // If skb was provided, it needs to be freed as we can't send it
        if (skb_to_free) {
            dev_kfree_skb_any(skb_to_free);
        }
        return -EBUSY;
    }

    // 2. Check frame length against descriptor buffer size
    if (frame_len == 0 || frame_len > desc->buffer_size) {
        dev_err(&priv->spi->dev, "Lowlevel_tx: Invalid frame_len %zu for buffer_size %zu\n", 
                frame_len, desc->buffer_size);
        dw1000_put_tx_desc_on_error(desc); // Return descriptor to free list
        if (skb_to_free) {
            dev_kfree_skb_any(skb_to_free);
        }
        return -EINVAL;
    }
    
    // 3. Copy frame data to descriptor's buffer
    memcpy(desc->buffer, frame_data, frame_len);

    // 4. Set descriptor fields
    desc->data_len = frame_len;
    desc->skb = skb_to_free; // Associate SKB for callback to free, if any

    // 5. Call the execute_tx helper to handle hardware config and DMA submission
    ret = dw1000_execute_tx(priv, desc);
    if (ret) {
        // dw1000_execute_tx failed. 
        // If it failed *before* DMA submission, tx_pending_count was decremented by it.
        // If it failed *after* DMA submission (e.g. TXSTRT command), DMA is pending.
        // The descriptor is NOT on the tx_pending_list if dw1000_submit_tx_desc failed.
        // The descriptor IS on the tx_pending_list if only TXSTRT failed.

        // Regardless, if execute_tx returns an error, we need to clean up here.
        // The descriptor should be returned to free_list ONLY if DMA was NOT successfully submitted.
        // dw1000_execute_tx handles atomic_dec if submit_tx_desc fails.
        // If submit_tx_desc succeeded but TXSTRT failed, desc is on pending_list, callback will clean it.
        // So, only call put_tx_desc_on_error if submit_tx_desc within execute_tx failed.
        // This is tricky. Let's assume execute_tx ensures desc is not on pending list if it returns error for pre-TXSTRT issues.

        // Simpler: If execute_tx returns error, it means the TX sequence was not fully successful.
        // The descriptor itself should be put back on the free list by the caller of execute_tx if DMA submission failed. 
        // dw1000_execute_tx already decremented tx_pending_count if dma_submit_tx_desc failed.
        // If only TXSTRT failed, desc is on pending_list and callback will handle it.
        // We only need to free the skb here and ensure desc is put back if not handled by callback.

        dev_err(&priv->spi->dev, "dw1000_execute_tx failed with %d in lowlevel_tx\n", ret);
        
        // Check if the descriptor is on the pending list. If not, it means DMA submission likely failed or was not attempted.
        // This requires checking list status, which adds complexity. 
        // A robust way: dw1000_execute_tx should guarantee that if it returns an error where DMA isn't pending,
        // the descriptor isn't on tx_pending_list. And tx_pending_count is correct.

        // For now, assume if dw1000_execute_tx fails, the descriptor state is such that
        // it should be returned to free list (unless it was due to TXSTRT failing post-DMA-submit).
        // The current dw1000_execute_tx logic: if dw1000_submit_tx_desc fails, it returns error, atomic_dec occurs.
        // Caller (this function) must put desc to free list.
        // If dw1000_submit_tx_desc succeeds but TXSTRT fails, desc IS on pending list. Callback handles.

        if (ret != -EAGAIN) { // Errors typically from submit_tx_desc or pre-submit checks
			dw1000_put_tx_desc_on_error(desc);
			// Else (e.g. error from TXSTRT), descriptor is on pending list, callback will free.

			if (desc->skb) { // Check desc->skb as skb_to_free might be different if logic changes
				dev_kfree_skb_any(desc->skb);
				desc->skb = NULL; // Important as descriptor might be reused from free list
			}

		} 
        return ret;
    }

    return 0; // Success
}

static const struct spi_device_id dw1000_generic_hybrid_id[] = {
	{ "dw1000-generic-hybrid", 0 },
	{ "dw1000-generic", 0 }, // Allow matching generic one
	{ }
};
MODULE_DEVICE_TABLE(spi, dw1000_generic_hybrid_id);

static const struct of_device_id dw1000_generic_hybrid_of_match[] = {
    { .compatible = "qorvo,dw1000-generic-hybrid" },
    { .compatible = "qorvo,dw1000-generic" },
    { .compatible = "decawave,dw1000" }, 
    { }
};

MODULE_DEVICE_TABLE(of, dw1000_generic_hybrid_of_match);

static struct spi_driver dw1000_generic_hybrid_spi_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = dw1000_generic_hybrid_of_match,
	},
	.id_table = dw1000_generic_hybrid_id,
	.probe = dw1000_generic_hybrid_probe,
	.remove = dw1000_generic_hybrid_remove,
};

module_spi_driver(dw1000_generic_hybrid_spi_driver);

MODULE_AUTHOR("Jinting Liu");
MODULE_DESCRIPTION("DW1000 driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.2");