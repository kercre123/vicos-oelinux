
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/acpi.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/of_gpio.h>


#define DEBUG

#include "bmi160.h"


static int bmi160_spi_probe(struct spi_device *spi)
{
	struct regmap *regmap;
	const struct spi_device_id *id;

	printk(KERN_ALERT "DEBUG: ENTER - Passed %s %d  spi_device: %p \n",__FUNCTION__,__LINE__, spi);
	id  = spi_get_device_id(spi);

	printk(KERN_ALERT "DEBUG: ENTER - Passed %s %d \n",__FUNCTION__,__LINE__);
	regmap = devm_regmap_init_spi(spi, &bmi160_regmap_config);
	printk(KERN_ALERT "DEBUG: ENTER - Passed %s %d \n",__FUNCTION__,__LINE__);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Failed to register spi regmap %d\n",
			(int)PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}
	printk(KERN_ALERT "DEBUG: ENTER - Passed %s %d \n",__FUNCTION__,__LINE__);
	return bmi160_core_probe(&spi->dev, regmap, id->name, true);
}







#ifdef CONFIG_OF //Open firmware must be defined for dts useage
static struct of_device_id qcom_spi_test_table[] = {
      { .compatible = "qcom,spi-test",}, //Compatible node must match                                   
                                        //dts
      { }, 
};
MODULE_DEVICE_TABLE(of, qcom_spi_test_table);
#else
#define qcom_spi_test_table NULL
#endif


static const struct spi_device_id bmi160_spi_id[] = {
	{"qcom_spi_test", 0},
	{}
};
MODULE_DEVICE_TABLE(spi, bmi160_spi_id);


//SPI Driver Info
static struct spi_driver spi_test_driver = {
	.probe = bmi160_spi_probe,
	.id_table = bmi160_spi_id,
	.driver = {
		.owner= THIS_MODULE,
		.of_match_table = qcom_spi_test_table,
		.name = "qcom_spi_test",
	}, 
};

module_spi_driver(spi_test_driver);
MODULE_DESCRIPTION("SPI TEST");
MODULE_LICENSE("GPL v2");
