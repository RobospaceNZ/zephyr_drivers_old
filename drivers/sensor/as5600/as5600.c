#define DT_DRV_COMPAT ams_as5600x

#include <device.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <drivers/i2c.h>
#define LOG_LEVEL LOG_LEVEL_INF
#include <logging/log.h>
LOG_MODULE_REGISTER(as5600, CONFIG_SENSOR_LOG_LEVEL);

#define AS5600_I2C_ADDR             0x36
// 2 bytes for raw angle and 1 byte for status
#define ANGLE_AND_STATUS_SIZE       3
#define AS5600_STATUS_ADDRESS       0x0B
#define AS5600_CONF_H_ADDRESS       0x08
#define AS5600_STATUS_OFFSET        0
#define AS5600_ANGLE_OFFSET         1
#define AS5600_LPM2                 0x02
#define AS5600_LPM3                 0x03

struct as5600_data 
{
	uint8_t as5600_data[ANGLE_AND_STATUS_SIZE];
    enum pm_device_state pm_state;
    //uint8_t i2c_address;
};

struct as5600_config {
    /*const struct device *i2c_master;
	struct gpio_dt_spec pwr_pin;*/
    uint8_t val;
};

// i2c_addr is the device address on the I2C bus
// addr is the first byte in the message (register address)
/*static int i2c_read_bytes(const struct device *i2c_dev, uint8_t addr, uint8_t *data, uint32_t num_bytes, uint8_t i2c_addr)
{
	struct i2c_msg msgs[2];

	// Send the address to read from
	msgs[0].buf = &addr;
	msgs[0].len = 1;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	// Read from device. STOP after this
	msgs[1].buf = data;
	msgs[1].len = num_bytes;
	msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	return i2c_transfer(i2c_dev, &msgs[0], 2, i2c_addr);
}

// i2c_addr is the device address on the I2C bus
// addr is the first byte in the message (register address)
static int i2c_write_bytes(const struct device *i2c_dev, uint8_t addr, uint8_t *data, uint32_t num_bytes, uint8_t i2c_addr)
{
	struct i2c_msg msgs[2];

	// Send the address to write to
	msgs[0].buf = &addr;
	msgs[0].len = 1;
	msgs[0].flags = I2C_MSG_WRITE;

	// Data to be written, and STOP after this
	msgs[1].buf = data;
	msgs[1].len = num_bytes;
	msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	return i2c_transfer(i2c_dev, &msgs[0], 2, i2c_addr);
}*/

static int as5600_sample_fetch(const struct device *dev,
				      enum sensor_channel chan)
{
	//const struct as5600_config *config = dev->config;
	//struct as5600_data *data = dev->data;
    //int error_code;

    if (chan == SENSOR_CHAN_ROTATION)
    {
        /*error_code = i2c_read_bytes(config->i2c_master, AS5600_STATUS_ADDRESS, data->as5600_data, sizeof(data->as5600_data), AS5600_I2C_ADDR);
        if (error_code)
        {
            LOG_ERR("Error reading from AS5600! error code (%d)\n", error_code);
            return error_code;
        }*/
    }
    else
    {
        return -ENOTSUP;
    }

	return 0;
}

static int as5600_channel_get(const struct device *dev,
				     enum sensor_channel chan,
				     struct sensor_value *val)
{
	/*struct as5600_data *data = dev->data;
    uint16_t angle;

	if (chan != SENSOR_CHAN_ROTATION) {
		return -ENOTSUP;
	}
    angle = data->as5600_data[AS5600_ANGLE_OFFSET];
    angle <<= 8;
    angle = data->as5600_data[AS5600_ANGLE_OFFSET+1];
    val->val1 = angle;
    val->val2 = data->as5600_data[AS5600_STATUS_OFFSET];
    */
    val->val1 = 0;
    val->val2 = 0;
	return 0;
}

static const struct sensor_driver_api as5600_api = {
	.sample_fetch = &as5600_sample_fetch,
	.channel_get = &as5600_channel_get,
};

static int as5600_init(const struct device *dev)
{
	//const struct as5600_config *config = dev->config;
    //struct as5600_data *data = dev->data;

	/*int ret;

	if (!device_is_ready(config->pwr_pin.port)) {
		LOG_ERR("AS5600 power pin not ready");
		return -ENODEV;
	}
    if (!config->i2c_master) {
		LOG_ERR("AS5600 device not found");
		return -ENODEV;
	}

    ret = gpio_pin_configure(config->pwr_pin.port, config->pwr_pin.pin, GPIO_OUTPUT_ACTIVE | GPIO_OPEN_SOURCE | GPIO_DS_ALT_HIGH | config->pwr_pin.dt_flags);
	if (ret < 0) {
		LOG_ERR("Error configuring AS5600 power pin\n");
	}
    gpio_pin_set(config->pwr_pin.port, config->pwr_pin.pin, true);
    data->pm_state = PM_DEVICE_STATE_ACTIVE;*/

	return 0;
}

#ifdef CONFIG_PM_DEVICE
int as5600_pm_ctrl(const struct device *dev, uint32_t ctrl_command,
		   enum pm_device_state *pm_state)
{
	//struct as5600_data *data = dev->data;
    //const struct as5600_config *config = dev->config;
	int ret = 0;
    //int error_code;
    //uint8_t i2c_value;

	/*if (ctrl_command == PM_DEVICE_STATE_SET) 
    {
        // Set power state
		if (*pm_state != data->pm_state) 
        {
			if (*pm_state == PM_DEVICE_STATE_ACTIVE) 
            {
				gpio_pin_set(config->pwr_pin.port, config->pwr_pin.pin, true);
                pm_device_state_set(config->i2c_master, PM_DEVICE_STATE_ACTIVE);
                k_msleep(100);
                i2c_value = AS5600_LPM2;
                error_code = i2c_write_bytes(config->i2c_master, AS5600_CONF_H_ADDRESS, &i2c_value, 1, AS5600_I2C_ADDR);
                if (error_code)
                {
                    LOG_ERR("Error enabling angle sensor run mode! error code (%d)\n", error_code);
                }
                else
                {
                    LOG_INF("AS5600 run mode\n");
                }
			}
			else if (*pm_state == PM_DEVICE_STATE_OFF)
            {
                gpio_pin_set(config->pwr_pin.port, config->pwr_pin.pin, false);
                pm_device_state_set(config->i2c_master, PM_DEVICE_STATE_OFF);
			}
            else
            {
                ret = -ENOTSUP;
            }

			// Store the new state
            if (!ret)
            {
                data->pm_state = *pm_state;
            }
		}
	}
	else 
    {
        // Get power state
		__ASSERT_NO_MSG(ctrl_command == PM_DEVICE_STATE_GET);
		*pm_state = data->pm_state;
	}*/
	return ret;
}
#endif // CONFIG_PM_DEVICE
//.pwr_pin = GPIO_DT_SPEC_INST_GET(inst, pwr_gpios),              
//.i2c_master = DEVICE_DT_GET(DT_INST_BUS(inst))                  


#define AS5600_INIT(inst)                                               \
	static struct as5600_data as5600_data_##inst;                       \
                                                                        \
	static const struct as5600_config as5600_config_##inst = {          \
		.val = 0                                                        \
	};                                                                  \
                                                                        \
	DEVICE_DT_INST_DEFINE(inst,                                         \
                    as5600_init,                                        \
                    as5600_pm_ctrl,                                     \
                    &as5600_data_##inst,                                \
                    &as5600_config_##inst,                              \
                    POST_KERNEL,                                        \
                    CONFIG_SENSOR_INIT_PRIORITY,                        \
                    &as5600_api);

DT_INST_FOREACH_STATUS_OKAY(AS5600_INIT)
