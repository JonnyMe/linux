// SPDX-License-Identifier: GPL-2.0-or-later

#include <asm/unaligned.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>

#define SHDW_FG_MSYS_SOC	0x61
#define SHDW_FG_VTG_NOW		0x69
#define SHDW_FG_CURR_NOW	0x6B
#define SHDW_FG_BATT_TEMP	0x6D

#define STATUS_3_REG		0x4B
#define CHG_HOLD_OFF_BIT	BIT(3)
#define CHG_TYPE_MASK		GENMASK(2, 1)
#define CHG_TYPE_SHIFT		1
#define BATT_NOT_CHG_VAL	0x0
#define BATT_PRE_CHG_VAL	0x1
#define BATT_FAST_CHG_VAL	0x2
#define BATT_TAPER_CHG_VAL	0x3

#define IRQ_E_REG		0x54
#define IRQ_STATUS_MASK		0x01
#define BITS_PER_IRQ		2

struct smb1360_battery {
	struct i2c_client	*client;
	struct device		*dev;
	struct regmap		*regmap;
	struct power_supply	*psy;
	struct mutex		read_write_lock;

	int battery_status;
	int hot_bat_decidegc;
	int cold_bat_decidegc;
};

static int smb1360_read_bytes(struct smb1360_battery *battery, int reg, u8 *val,
			      u8 bytes)
{
	s32 ret;

	ret = regmap_bulk_read(battery->regmap, reg, val, bytes);
	if (ret) {
		dev_err(&battery->client->dev,
			"failed to read from %d registry\n", reg);
		return ret;
	}

	return 0;
}

static int smb1360_get_prop_batt_overvoltage(struct smb1360_battery *battery)
{
	int ret = 0;
	u8 reg, usbin_ov = 0;

	ret = smb1360_read_bytes(battery, IRQ_E_REG, &reg, 1);
	if (ret)
		return ret;

	usbin_ov = reg & (IRQ_STATUS_MASK << (1 * BITS_PER_IRQ));
	return !!usbin_ov;
}

static int smb1360_get_prop_batt_health(struct smb1360_battery *battery)
{
	int ret = 0;
	union power_supply_propval prop_temp = {0, };

	if (smb1360_get_prop_batt_overvoltage(battery))
		return POWER_SUPPLY_HEALTH_OVERVOLTAGE;

	ret = power_supply_get_property(battery->psy, POWER_SUPPLY_PROP_TEMP, &prop_temp);
	if (ret)
		return POWER_SUPPLY_HEALTH_UNKNOWN;

	if (prop_temp.intval > battery->hot_bat_decidegc)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (prop_temp.intval < battery->cold_bat_decidegc)
		return POWER_SUPPLY_HEALTH_COLD;

	return POWER_SUPPLY_HEALTH_GOOD;
}

static int smb1360_get_prop_batt_status(struct smb1360_battery *battery)
{
	int ret;
	u8 reg;
	u32 chg_type;

	ret = smb1360_read_bytes(battery, STATUS_3_REG, &reg, 1);
	if (ret)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	if (reg & CHG_HOLD_OFF_BIT)
		return POWER_SUPPLY_STATUS_NOT_CHARGING;

	chg_type = (reg & CHG_TYPE_MASK) >> CHG_TYPE_SHIFT;
	if (chg_type == BATT_NOT_CHG_VAL)
		return POWER_SUPPLY_STATUS_DISCHARGING;

	return POWER_SUPPLY_STATUS_CHARGING;
}

static int smb1360_get_prop_charge_type(struct smb1360_battery *battery)
{
	int ret;
	u8 reg;
	u32 chg_type;

	ret = smb1360_read_bytes(battery, STATUS_3_REG, &reg, 1);
	if (ret)
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	chg_type = (reg & CHG_TYPE_MASK) >> CHG_TYPE_SHIFT;
	switch (chg_type) {
	case BATT_NOT_CHG_VAL:
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	case BATT_FAST_CHG_VAL:
	case BATT_TAPER_CHG_VAL:
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	case BATT_PRE_CHG_VAL:
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	default:
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	}
}

static int smb1360_get_prop_batt_capacity(struct smb1360_battery *battery)
{
	int ret, soc = 0;
	u8 reg;

	ret = smb1360_read_bytes(battery, SHDW_FG_MSYS_SOC, &reg, 1);
	if (ret)
		return ret;

	soc = DIV_ROUND_CLOSEST((100 * reg), 255);
	soc = clamp(soc, 0, 100);

	return soc;
}

static int smb1360_get_prop_voltage_now(struct smb1360_battery *battery)
{
	u8 reg[2];
	int ret, temp = 0;

	ret = smb1360_read_bytes(battery, SHDW_FG_VTG_NOW, reg, 2);
	if (ret)
		return ret;

	temp = get_unaligned_le16(reg);
	temp = div_u64(temp * 5000, 0x7FFF) * 1000;

	return temp;
}

static int smb1360_get_prop_current_now(struct smb1360_battery *battery)
{
	u8 reg[2];
	int ret, temp = 0;

	ret = smb1360_read_bytes(battery, SHDW_FG_CURR_NOW, reg, 2);
	if (ret)
		return ret;

	temp = (s16) get_unaligned_le16(reg);
	temp = div_s64(temp * 2500, 0x7FFF) * 1000;

	return temp;
}

static int smb1360_get_prop_batt_temp(struct smb1360_battery *battery)
{
	u8 reg[2];
	int ret, temp = 0;

	ret = smb1360_read_bytes(battery, SHDW_FG_BATT_TEMP, reg, 2);
	if (ret)
		return ret;

	temp = get_unaligned_le16(reg);
	temp = div_u64(temp * 625, 10000UL); /* temperature in kelvin */
	temp = (temp - 273) * 10; /* temperature in decideg */

	return temp;
}

static int smb1360_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct smb1360_battery *battery = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = smb1360_get_prop_batt_health(battery);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = smb1360_get_prop_batt_status(battery);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = smb1360_get_prop_charge_type(battery);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = smb1360_get_prop_batt_capacity(battery);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = smb1360_get_prop_voltage_now(battery);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = smb1360_get_prop_current_now(battery);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = smb1360_get_prop_batt_temp(battery);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property smb1360_battery_props[] = {
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
};

static const struct regmap_config smb1360_battery_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
};

static int battery_status_handler(struct smb1360_battery *battery,
				  int battery_status)
{
	if (battery->battery_status != battery_status) {
		power_supply_changed(battery->psy);
	}

	battery->battery_status = battery_status;

	return 0;
}

static irqreturn_t smb1360_bat_irq(int irq, void *data)
{
	struct smb1360_battery *battery = data;
	int battery_status;

	battery_status = smb1360_get_prop_batt_status(battery);
	battery_status_handler(battery, battery_status);

	return IRQ_HANDLED;
}

static const struct power_supply_desc smb1360_battery_desc = {
	.name		= "smb1360-battery",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.get_property	= smb1360_battery_get_property,
	.properties	= smb1360_battery_props,
	.num_properties	= ARRAY_SIZE(smb1360_battery_props),
};

static int smb1360_parse_properties(struct smb1360_battery *battery)
{
	int ret = 0;
	struct device *dev = &battery->client->dev;

	ret = device_property_read_u32(dev, "smb1360,hot-bat-decidegc",
				       &battery->hot_bat_decidegc);
	if (ret)
		return -EINVAL;

	ret = device_property_read_u32(dev, "smb1360,cold-bat-decidegc",
				       &battery->cold_bat_decidegc);
	if (ret)
		return -EINVAL;

	if (device_property_read_bool(dev, "smb1360,cold-bat-below-zero"))
		battery->cold_bat_decidegc = -battery->cold_bat_decidegc;

	return 0;
}

static int smb1360_fg_probe(struct i2c_client *client)
{
	struct power_supply_config psy_cfg = {};
	struct smb1360_battery *battery;
	int ret;

	battery = devm_kzalloc(&client->dev, sizeof(*battery), GFP_KERNEL);
	if (!battery) {
		dev_err(&client->dev, "failed to allocate memory\n");
		return -EINVAL;
	}

	mutex_init(&battery->read_write_lock);

	battery->client = client;

	ret = smb1360_parse_properties(battery);
	if (ret) {
		dev_err(&client->dev, "failed to parse dt properties\n");
		return ret;
	}

	battery->regmap = devm_regmap_init_i2c(client,
					       &smb1360_battery_regmap_config);
	if (IS_ERR(battery->regmap)) {
		dev_err(&client->dev, "failed to initialize regmap\n");
		return -EINVAL;
	}

	i2c_set_clientdata(client, battery);
	psy_cfg.drv_data = battery;

	battery->psy = power_supply_register(&client->dev,
					     &smb1360_battery_desc, &psy_cfg);
	if (IS_ERR(battery->psy)) {
		dev_err(&client->dev, "failed to register power supply\n");
		ret = PTR_ERR(battery->psy);
		return ret;
	}

	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
					       smb1360_bat_irq, IRQF_ONESHOT, NULL, battery);
		if (ret) {
			dev_err(&client->dev, "request irq %d failed\n", client->irq);
			return ret;
		}

		enable_irq_wake(client->irq);
	}

	return 0;
}

static int smb1360_suspend(struct device *dev)
{
	struct smb1360_battery *battery = dev_get_drvdata(dev);

	if (battery->client->irq) {
		disable_irq(battery->client->irq);
		enable_irq_wake(battery->client->irq);
	}

	return 0;
}

static int smb1360_resume(struct device *dev)
{
	struct smb1360_battery *battery = dev_get_drvdata(dev);

	if (battery->client->irq) {
		disable_irq_wake(battery->client->irq);
		enable_irq(battery->client->irq);
	}

	return 0;
}

static SIMPLE_DEV_PM_OPS(smb1360_pm_ops, smb1360_suspend, smb1360_resume);

#ifdef CONFIG_OF
static struct of_device_id smb1360_match_table[] = {
	{ .compatible = "qcom,smb1360" },
	{ },
};
MODULE_DEVICE_TABLE(of, smb1360_match_table);
#endif

static struct i2c_driver smb1360_fg_driver = {
	.driver = {
		.name = "smb1360",
		.of_match_table = of_match_ptr(smb1360_match_table),
		.pm = &smb1360_pm_ops,
	},
	.probe_new = smb1360_fg_probe,
};

module_i2c_driver(smb1360_fg_driver);

MODULE_DESCRIPTION("Fuel-Gauge Driver for Qualcomm SMB1360");
MODULE_LICENSE("GPL");
