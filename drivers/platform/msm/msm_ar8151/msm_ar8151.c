 /* Copyright (C) 2017 Facebook Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/msm_pcie.h>
#include <asm/dma-iommu.h>
#include <linux/msm-bus.h>
#include <linux/iommu.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <soc/qcom/subsystem_restart.h>
#include <soc/qcom/subsystem_notif.h>
#include <soc/qcom/ramdump.h>
#include <soc/qcom/memory_dump.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include "msm_ar8151.h"

#define AR8151_VENDOR (0x1969)
#define AR8151_DEVICE (0x1083)

#define SMMU_BASE	0x10000000 /* Device address range base */
#define SMMU_SIZE	((SZ_1G * 4ULL) - SMMU_BASE)

#define PM_OPT_SUSPEND (MSM_PCIE_CONFIG_NO_CFG_RESTORE | \
			MSM_PCIE_CONFIG_LINKDOWN)
#define PM_OPT_RESUME MSM_PCIE_CONFIG_NO_CFG_RESTORE

#define DISABLE_PCIE_L1_MASK 0xFFFFFFFD
#define PCIE20_CAP_LINKCTRLSTATUS 0x80

struct device;

struct msm_ar8151_ctx {
	struct list_head list;
	struct device *dev; /* for platform device */

	/* pci device */
	u32 rc_index; /* PCIE root complex index */
	struct pci_dev *pcidev;
	struct pci_saved_state *pristine_state;

	/* SMMU */
	bool use_smmu; /* have SMMU enabled? */
	int smmu_bypass;
	int smmu_fast_map;
	struct dma_iommu_mapping *mapping;

	struct ar8151_platform_ops ops;
};

static LIST_HEAD(dev_list);

static struct msm_ar8151_ctx *pcidev2ctx(struct pci_dev *pcidev)
{
	struct msm_ar8151_ctx *ctx;

	list_for_each_entry(ctx, &dev_list, list) {
		if (ctx->pcidev == pcidev)
			return ctx;
	}
	return NULL;
}

static int ops_suspend(void *handle)
{
	int rc;
	struct msm_ar8151_ctx *ctx = handle;
	struct pci_dev *pcidev;

	pr_info("%s(%p)\n", __func__, handle);
	if (!ctx) {
		pr_err("No context\n");
		return -ENODEV;
	}
	pcidev = ctx->pcidev;
	rc = pci_save_state(pcidev);
	if (rc) {
		dev_err(ctx->dev, "pci_save_state failed :%d\n", rc);
		return rc;
	}
	rc = msm_pcie_pm_control(MSM_PCIE_SUSPEND, pcidev->bus->number,
				 pcidev, NULL, PM_OPT_SUSPEND);
	if (rc) {
		dev_err(ctx->dev, "msm_pcie_pm_control(SUSPEND) failed :%d\n",
			rc);
		return rc;
	}

	return rc;
}

static int ops_resume(void *handle)
{
	int rc;
	struct msm_ar8151_ctx *ctx = handle;
	struct pci_dev *pcidev;
	u32 val;

	pr_info("%s(%p)\n", __func__, handle);
	if (!ctx) {
		pr_err("No context\n");
		return -ENODEV;
	}

	pcidev = ctx->pcidev;

	rc = msm_pcie_pm_control(MSM_PCIE_RESUME, pcidev->bus->number,
				 pcidev, NULL, PM_OPT_RESUME);
	if (rc) {
		dev_err(ctx->dev, "msm_pcie_pm_control(RESUME) failed :%d\n",
			rc);
		return rc;
	}
	rc = msm_pcie_recover_config(pcidev);
	if (rc) {
		dev_err(ctx->dev, "msm_pcie_recover_config failed :%d\n",
			rc);
		goto err_suspend_rc;
	}

	/* Disable L1 */
	rc = pci_read_config_dword(ctx->pcidev,
				   PCIE20_CAP_LINKCTRLSTATUS, &val);
	if (rc) {
		dev_err(ctx->dev,
			"reading PCIE20_CAP_LINKCTRLSTATUS failed:%d\n",
			rc);
		goto err_suspend_rc;
	}
	val &= DISABLE_PCIE_L1_MASK; /* disable bit 1 */
	dev_dbg(ctx->dev, "writing PCIE20_CAP_LINKCTRLSTATUS (val 0x%x)\n",
		val);
	rc = pci_write_config_dword(ctx->pcidev,
				    PCIE20_CAP_LINKCTRLSTATUS, val);
	if (rc) {
		dev_err(ctx->dev,
			"writing PCIE20_CAP_LINKCTRLSTATUS (val 0x%x) failed:%d\n",
			val, rc);
		goto err_suspend_rc;
	}

	return 0;

err_suspend_rc:
	msm_pcie_pm_control(MSM_PCIE_SUSPEND, pcidev->bus->number,
			    pcidev, NULL, PM_OPT_SUSPEND);
	return rc;
}

static int msm_ar8151_smmu_init(struct msm_ar8151_ctx *ctx)
{
	int atomic_ctx = 1;
	int rc;

	if (!ctx->use_smmu)
		return 0;

	dev_info(ctx->dev, "Initialize SMMU, bypass = %d, fastmap = %d\n",
		 ctx->smmu_bypass, ctx->smmu_fast_map);

	ctx->mapping = arm_iommu_create_mapping(&platform_bus_type,
						SMMU_BASE, SMMU_SIZE);
	if (IS_ERR_OR_NULL(ctx->mapping)) {
		rc = PTR_ERR(ctx->mapping) ?: -ENODEV;
		dev_err(ctx->dev, "Failed to create IOMMU mapping (%d)\n", rc);
		return rc;
	}

	rc = iommu_domain_set_attr(ctx->mapping->domain,
				   DOMAIN_ATTR_ATOMIC,
				   &atomic_ctx);
	if (rc) {
		dev_err(ctx->dev, "Set atomic attribute to SMMU failed (%d)\n",
			rc);
		goto release_mapping;
	}

	if (ctx->smmu_bypass) {
		rc = iommu_domain_set_attr(ctx->mapping->domain,
					   DOMAIN_ATTR_S1_BYPASS,
					   &ctx->smmu_bypass);
		if (rc) {
			dev_err(ctx->dev, "Set bypass attribute to SMMU failed (%d)\n",
				rc);
			goto release_mapping;
		}
	} else if (ctx->smmu_fast_map) {
		rc = iommu_domain_set_attr(ctx->mapping->domain,
					   DOMAIN_ATTR_FAST,
					   &ctx->smmu_fast_map);
		if (rc) {
			dev_err(ctx->dev, "Set fast attribute to SMMU failed (%d)\n",
				rc);
			goto release_mapping;
		}
	}

	rc = arm_iommu_attach_device(&ctx->pcidev->dev, ctx->mapping);
	if (rc) {
		dev_err(ctx->dev, "arm_iommu_attach_device failed (%d)\n", rc);
		goto release_mapping;
	}
	dev_info(ctx->dev, "attached to IOMMU\n");

	return 0;
release_mapping:
	arm_iommu_release_mapping(ctx->mapping);
	ctx->mapping = NULL;
	return rc;
}

static int msm_ar8151_probe(struct platform_device *pdev)
{
	struct msm_ar8151_ctx *ctx;
	struct device *dev = &pdev->dev;
	struct device_node *of_node = dev->of_node;
	struct device_node *rc_node;
	struct pci_dev *pcidev = NULL;
	int rc;
	u32 val;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = dev;

	/*== parse ==*/
	/* Information pieces:
	 * - of_node stands for "ar8151":
	 *	ar8151: qcom,ar8151 {
	 *	compatible = "qcom,ar8151";
	 *	qcom,pcie-parent = <&pcie1>;
	 *};
	 * rc_node stands for "qcom,pcie", selected entries:
	 * cell-index = <1>; (ctx->rc_index)
	 */

	rc_node = of_parse_phandle(of_node, "qcom,pcie-parent", 0);
	if (!rc_node) {
		dev_err(ctx->dev, "Parent PCIE device not found\n");
		return -EINVAL;
	}
	rc = of_property_read_u32(rc_node, "cell-index", &ctx->rc_index);
	if (rc < 0) {
		dev_err(ctx->dev, "Parent PCIE device index not found\n");
		return -EINVAL;
	}
	ctx->use_smmu = true;

	ctx->smmu_bypass = 1;
	ctx->smmu_fast_map = 0;

	/* enumerate it on PCIE */
	rc = msm_pcie_enumerate(ctx->rc_index);
	if (rc < 0) {
		dev_err(ctx->dev, "Parent PCIE enumeration failed\n");
		goto out_rc;
	}
	/* search for PCIE device in our domain */
	do {
		pcidev = pci_get_device(AR8151_VENDOR, AR8151_DEVICE, pcidev);
		if (!pcidev)
			break;

		if (pci_domain_nr(pcidev->bus) == ctx->rc_index)
			break;
	} while (true);
	if (!pcidev) {
		rc = -ENODEV;
		dev_err(ctx->dev, "Ar8151 device %4x:%4x not found\n",
			AR8151_VENDOR, AR8151_DEVICE);
		goto out_rc;
	}
	ctx->pcidev = pcidev;

	/* Disable L1 */
	rc = pci_read_config_dword(ctx->pcidev,
				   PCIE20_CAP_LINKCTRLSTATUS, &val);
	if (rc) {
		dev_err(ctx->dev,
			"reading PCIE20_CAP_LINKCTRLSTATUS failed:%d\n",
			rc);
		goto out_rc;
	}
	val &= DISABLE_PCIE_L1_MASK; /* disable bit 1 */
	dev_dbg(ctx->dev, "writing PCIE20_CAP_LINKCTRLSTATUS (val 0x%x)\n",
		 val);
	rc = pci_write_config_dword(ctx->pcidev,
				    PCIE20_CAP_LINKCTRLSTATUS, val);
	if (rc) {
		dev_err(ctx->dev,
			"writing PCIE20_CAP_LINKCTRLSTATUS (val 0x%x) failed:%d\n",
			val, rc);
		goto out_rc;
	}

	rc = pci_save_state(pcidev);
	if (rc) {
		dev_err(ctx->dev, "pci_save_state failed :%d\n", rc);
		goto out_rc;
	}
	ctx->pristine_state = pci_store_saved_state(pcidev);

	/* report */
	dev_info(ctx->dev, "msm_ar8151 discovered. %p {\n"
		 "  rc_index = %d\n"
		 "  use_smmu = %d\n"
		 "  pcidev = %p\n"
		 "}\n", ctx, ctx->rc_index, ctx->use_smmu, ctx->pcidev);

	platform_set_drvdata(pdev, ctx);
	device_disable_async_suspend(&pcidev->dev);

	list_add_tail(&ctx->list, &dev_list);
	ops_suspend(ctx);

	return 0;
out_rc:
	return rc;
}

static int msm_ar8151_remove(struct platform_device *pdev)
{
	struct msm_ar8151_ctx *ctx = platform_get_drvdata(pdev);

	list_del(&ctx->list);
	dev_info(ctx->dev, "%s: pdev %p pcidev %p\n", __func__, pdev,
		 ctx->pcidev);
	kfree(ctx->pristine_state);

	pci_dev_put(ctx->pcidev);

	return 0;
}

static const struct of_device_id msm_ar8151_of_match[] = {
	{ .compatible = "qcom,ar8151", },
	{},
};

static struct platform_driver msm_ar8151_driver = {
	.driver = {
		.name = "msm_ar8151",
		.of_match_table = msm_ar8151_of_match,
	},
	.probe = msm_ar8151_probe,
	.remove = msm_ar8151_remove,
};
module_platform_driver(msm_ar8151_driver);

static void ops_uninit(void *handle)
{
	struct msm_ar8151_ctx *ctx = (struct msm_ar8151_ctx *)handle;

	if (ctx->use_smmu) {
		arm_iommu_detach_device(&ctx->pcidev->dev);
		arm_iommu_release_mapping(ctx->mapping);
		ctx->mapping = NULL;
	}

	ops_suspend(ctx);
}

void *msm_ar8151_dev_init(struct device *dev)
{
	struct pci_dev *pcidev = to_pci_dev(dev);
	struct msm_ar8151_ctx *ctx = pcidev2ctx(pcidev);

	if (!ctx) {
		pr_err("Context not found for pcidev %p\n", pcidev);
		return NULL;
	}

	/* smmu */
	if (msm_ar8151_smmu_init(ctx))
		return NULL;

	/* fill ops */
	memset(&ctx->ops, 0, sizeof(ctx->ops));
	ctx->ops.suspend = ops_suspend;
	ctx->ops.resume = ops_resume;
	ctx->ops.uninit = ops_uninit;

	return ctx;
}
EXPORT_SYMBOL(msm_ar8151_dev_init);

int msm_ar8151_modinit(void)
{
	struct msm_ar8151_ctx *ctx;

	ctx = list_first_entry_or_null(&dev_list, struct msm_ar8151_ctx,
				       list);

	if (!ctx) {
		pr_err("Context not found\n");
		return -EINVAL;
	}

	if (ctx->pristine_state) {
		/* in old kernels, pci_load_saved_state() is not exported;
		 * so use pci_load_and_free_saved_state()
		 * and re-allocate ctx->saved_state again
		 */
		pci_load_and_free_saved_state(ctx->pcidev,
					      &ctx->pristine_state);
		ctx->pristine_state = pci_store_saved_state(ctx->pcidev);
	}

	return ops_resume(ctx);
}
EXPORT_SYMBOL(msm_ar8151_modinit);

void msm_ar8151_modexit(void)
{
	struct msm_ar8151_ctx *ctx;

	ctx = list_first_entry_or_null(&dev_list, struct msm_ar8151_ctx,
				       list);

	if (!ctx) {
		pr_err("Context not found\n");
		return;
	}

	/* ops_uninit(ctx); */
}

struct ar8151_platform_ops *msm_ar8151_get_ops(struct pci_dev *pcidev)
{
	struct msm_ar8151_ctx *ctx = pcidev2ctx(pcidev);

	if (!ctx)
		return NULL;

	return &ctx->ops;
}
EXPORT_SYMBOL(msm_ar8151_get_ops);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Platform driver for qcom ar8151 card");

