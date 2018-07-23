/*
 * HLW8012 driver
 *
 * Copyright (c) Felix Froehling <felix.froehling1@gmail.com> 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/sysfs.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/wait.h>
#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/timekeeping.h>

#include <linux/iio/iio.h>

#define DRIVER_NAME	"hlw8012"

#define HLW8012_MEASURE_TIME	1000
#define HLW8012_MEASURE_DELAY	2500

#define V_REF			2.43  
#define R_CURRENT		0.001 
#define R_VOLTAGE		2821  
#define F_OSC			3579000

//MATH constants
#define N_DECIMAL_POINTS_PRECISION (1000) // n = 3. Three decimal points.


struct hlw8012 {
	struct device			*dev;

	int				gpio_sel; //Selection for Power of current on cf1
	int				gpio_cf1; //Data output dependend on gpio sel (voltage or current)
	int				gpio_cf;  //Actve power

	struct completion 		completion;

	struct mutex			lock_cf;
	struct mutex			lock_cf1;

	int 				frequency_cf1;
	int 				frequency_cf;
	int				irq_cf1;
	int				irq_cf;
};

/*Interupt Handler CF1 */
static irqreturn_t hlw8012_handle_irq_cf1(int irq, void *data){
	struct iio_dev *iio = data;
	struct hlw8012 *hlw8012 = iio_priv(iio);

	hlw8012->frequency_cf1 += 1;
	
	return IRQ_HANDLED;
}

/*Interupt Handler CF*/
static irqreturn_t hlw8012_handle_irq_cf(int irq, void *data){
	struct iio_dev *iio = data;
	struct hlw8012 *hlw8012 = iio_priv(iio);

	hlw8012->frequency_cf += 1;
	
	return IRQ_HANDLED;
}

/*Set sel PIN for mode selection*/
static void set_sel_pin(struct hlw8012 *hlw8012, const struct iio_chan_spec *chan){
	int value = 0;
	
	if(chan->type == IIO_VOLTAGE){
		value = 0;
	}
	else if(chan->type == IIO_CURRENT){
		value = 1;
	}
	else{
		return;
	}

	gpio_set_value(hlw8012->gpio_sel, value);
	msleep(HLW8012_MEASURE_DELAY);	
	
}

/*relase all interupts */
static void releaseInterrupt(struct hlw8012 *hlw8012, struct iio_dev *iio_dev, const struct iio_chan_spec *chan){
	if(chan->type == IIO_VOLTAGE || chan->type == IIO_CURRENT){ //
		free_irq(hlw8012->irq_cf1, iio_dev);
	}
	else{
		free_irq(hlw8012->irq_cf, iio_dev);
	}
}

/*aquire lock and init interupts */
static int setLockAndInterrupt(struct hlw8012 *hlw8012, struct iio_dev *iio_dev, const struct iio_chan_spec *chan){
	int ret = 0;

	//Set Interrupt Handler
	if(chan->type == IIO_VOLTAGE || chan->type == IIO_CURRENT){ //
		mutex_lock(&hlw8012->lock_cf1); //Get lock on cf1
		hlw8012->frequency_cf1 = 0; //Init Edges to 0

		ret = request_irq(hlw8012->irq_cf1, hlw8012_handle_irq_cf1,
			IRQF_TRIGGER_RISING,
			iio_dev->name, iio_dev);
	}
	else{
		mutex_lock(&hlw8012->lock_cf); //Get lock on cf
		hlw8012->frequency_cf = 0; //Init Edges to 0

		ret = request_irq(hlw8012->irq_cf, hlw8012_handle_irq_cf,
			IRQF_TRIGGER_RISING,
			iio_dev->name, iio_dev);
	}

	return ret;
}

/*release lock */
static void releaseMutex(struct hlw8012 *hlw8012, const struct iio_chan_spec *chan){
	if(chan->type == IIO_VOLTAGE || chan->type == IIO_CURRENT){ //
		mutex_unlock(&hlw8012->lock_cf1); //Get lock on cf1
	}
	else{
		mutex_unlock(&hlw8012->lock_cf); //Get lock on cf
	}
}

/*calculate final result */
static int calculateResult(struct hlw8012 *hlw8012, const struct iio_chan_spec *chan){
	int frequency, length_of_pulse, multiplier, voltage, curr, power;

	if(chan->type == IIO_VOLTAGE){
		frequency = (int) (hlw8012->frequency_cf1 * (1000 / HLW8012_MEASURE_TIME)); //Transfer Measure to one second
		if(frequency == 0){
			return 0;
		}

		length_of_pulse = (int) (1000 / frequency); 
		multiplier = (int) ( 1000000.0 * 512 * V_REF * R_VOLTAGE / 2.0 / F_OSC );
		voltage = (int) (multiplier / length_of_pulse / 2);
		return voltage;
	}
	else if(chan->type == IIO_CURRENT){
		frequency = hlw8012->frequency_cf1 * (1000 / HLW8012_MEASURE_TIME); //Transfer Measure to one second
		if(frequency == 0){
			return 0;
		}

		length_of_pulse = (int)(1000 / frequency); 
		multiplier = (int)(( 1000000.0 * 512 * V_REF / R_CURRENT / 24.0 / F_OSC ));
		curr = (int) (multiplier / length_of_pulse / 2);
		return curr;
	}
	else { // Power
		frequency = hlw8012->frequency_cf * (1000 / HLW8012_MEASURE_TIME); //Transfer Measure to one second
		if(frequency == 0){
			return 0 ;
		}

		length_of_pulse = (int)(1000 / frequency); 
		multiplier = (int)(( 1000000.0 * 128 * V_REF * V_REF * R_VOLTAGE / R_CURRENT / 48.0 / F_OSC ));
		power = (int) (multiplier / length_of_pulse / 2);
		return power;
	}
}

/*read driver */
static int hlw8012_read_raw(struct iio_dev *iio_dev,
			  const struct iio_chan_spec *chan,
			int *val, int *val2, long m)
{
	struct hlw8012 *hlw8012 = iio_priv(iio_dev);
	int ret = IIO_VAL_INT;

	//First set Pin
	set_sel_pin(hlw8012, chan);	

	//Get Lock and init interrupts
	if (setLockAndInterrupt(hlw8012, iio_dev, chan)){
		ret = -EINVAL;
		goto err;
	}

	//Sleep
	msleep(HLW8012_MEASURE_TIME);	

	//Release interupt
	releaseInterrupt(hlw8012, iio_dev, chan);

	//Umrechnen
	(*val) = calculateResult(hlw8012, chan);
		
err:
	releaseMutex(hlw8012, chan);
	return ret;
}

static const struct iio_info hlw8012_iio_info = {
	.driver_module		= THIS_MODULE,
	.read_raw		= hlw8012_read_raw,
};

static const struct iio_chan_spec hlw8012_chan_spec[] = {
	{ .type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED), },
	{ .type = IIO_CURRENT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED), },
	{ .type = IIO_POWER,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED), }
};

static const struct of_device_id hlw8012_dt_ids[] = {
	{ .compatible = "hlw8012", },
	{ }
};
MODULE_DEVICE_TABLE(of, hlw8012_dt_ids);

static int hlw8012_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct hlw8012 *hlw8012;
	struct iio_dev *iio;
	int read;

	//init
	iio = devm_iio_device_alloc(dev, sizeof(*hlw8012));
	if (!iio) {
		dev_err(dev, "Failed to allocate IIO device\n");
		return -ENOMEM;
	}

	hlw8012 = iio_priv(iio);
	hlw8012->dev = dev;

	//sel	
	read = of_get_named_gpio(node, "sel-gpio", 0);
	if (read < 0)
		return read;
	hlw8012->gpio_sel = read;

	//cf1
	read = of_get_named_gpio(node, "cf1-gpio", 0);
	if (read < 0)
		return read;
	hlw8012->gpio_cf1 = read;

	//cf
	read = of_get_named_gpio(node, "cf-gpio", 0);
	if (read < 0)
		return read;
	hlw8012->gpio_cf = read;

	gpio_direction_output(hlw8012->gpio_sel, 0);
	gpio_direction_input(hlw8012->gpio_cf1);
	gpio_direction_input(hlw8012->gpio_cf);

	hlw8012->irq_cf1 = gpio_to_irq(hlw8012->gpio_cf1);
	if (hlw8012->irq_cf1 < 0) {
		dev_err(dev, "GPIO %d has no interrupt\n", hlw8012->gpio_cf1);
		return -EINVAL;
	}

	hlw8012->irq_cf = gpio_to_irq(hlw8012->gpio_cf);
	if (hlw8012->irq_cf < 0) {
		dev_err(dev, "GPIO %d has no interrupt\n", hlw8012->gpio_cf);
		return -EINVAL;
	}

	platform_set_drvdata(pdev, iio);

	init_completion(&hlw8012->completion);
	mutex_init(&hlw8012->lock_cf1);
	mutex_init(&hlw8012->lock_cf);
	iio->name = pdev->name;
	iio->dev.parent = &pdev->dev;
	iio->info = &hlw8012_iio_info;
	iio->modes = INDIO_DIRECT_MODE;
	iio->channels = hlw8012_chan_spec;
	iio->num_channels = ARRAY_SIZE(hlw8012_chan_spec);


	return devm_iio_device_register(dev, iio);
}

static struct platform_driver hwl8012_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.of_match_table = hlw8012_dt_ids,
	},
	.probe  = hlw8012_probe,
};

module_platform_driver(hwl8012_driver);

MODULE_AUTHOR("Felix Froehling <felix.froehling1@gmail.com>");
MODULE_DESCRIPTION("HLW8012 Powermeter driver");
MODULE_LICENSE("GPL v2");
