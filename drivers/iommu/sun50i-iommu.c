// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Yangtao Li <tiny.windzz@gmail.com>
 */

#include <linux/clk.h>
#include <linux/dma-iommu.h>
#include <linux/interrupt.h>
#include <linux/iommu.h>
#include <linux/kmemleak.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#define SUN50I_IOMMU_EN				0x020
#define SUN50I_IOMMU_BYPASS			0x030
#define SUN50I_IOMMU_TTB			0x050
#define SUN50I_IOMMU_TE				0x060
#define SUN50I_IOMMU_TP				0x070
#define SUN50I_IOMMU_TIA			0x090
#define SUN50I_IOMMU_TIAM			0x094
#define SUN50I_IOMMU_TIE			0x098
#define SUN50I_IOMMU_PCIA			0x0A0
#define SUN50I_IOMMU_PCIE			0x0A8
#define SUN50I_IOMMU_IE				0x100
#define SUN50I_IOMMU_IC				0x104
#define SUN50I_IOMMU_IS				0x108
#define SUN50I_IOMMU_IEA6			0x130
#define SUN50I_IOMMU_IEA7			0x134

#define SUN50I_IOMMU_ENABLE			BIT(0)
#define SUN50I_IOMMU_BYPASS_VAL			GENMASK(5, 0)
#define SUN50I_IOMMU_TLB_ENABLE			(GENMASK(17, 16) | \
						 GENMASK(5, 0))
#define SUN50I_IOMMU_TLB_PREFETCH		GENMASK(5, 0)
#define SUN50I_IOMMU_TLB_IVLD_ADDR_MASK		GENMASK(31, 12)
#define SUN50I_IOMMU_INT_ENABLE			GENMASK(17, 16)
#define SUN50I_IOMMU_TLB_IVLD_ENABLE		BIT(0)
#define SUN50I_IOMMU_PC_IVLD_ENABLE		BIT(0)

#define L1_PAGETABLE_INVALID			BIT(16)

#define SUNXI_MMU_POLL_TIMEOUT_US		2000 /* 2ms */

struct sunxi_iommu {
	struct device		*dev;
	struct clk		*clk;
	struct regmap		*regmap;
	struct reset_control	*reset;
	struct iommu_group	*group;
	struct iommu_domain	*domain;
	struct iommu_device	iommu;
};

struct sunxi_iommu_domain {
	dma_addr_t		dt_dma;
	u32			*dt;
	struct kmem_cache	*pt_kmem_cache;
	spinlock_t		dt_lock;
	struct sunxi_iommu	*iommu;
	struct			iommu_domain domain;
};

static struct device *dma_dev;

static inline void sunxi_table_flush(dma_addr_t dma, unsigned int count)
{
	size_t size = count * sizeof(u32);	/* count of u32 entry */

	dma_sync_single_for_device(dma_dev, dma, size, DMA_TO_DEVICE);
}

#define NUM_DT_ENTRIES 4096
#define NUM_PT_ENTRIES 256

#define DT_SIZE (NUM_DT_ENTRIES * sizeof(u32))
#define PT_SIZE (NUM_PT_ENTRIES * sizeof(u32))

#define SPAGE_SIZE (1 << 12)

/*
 * Support mapping any size that fits in one page table:
 *   4 KiB to 1 MiB
 */
#define SUNXI_IOMMU_PGSIZE_BITMAP GENMASK(20, 12)

/*
 * The Allwinner h6 iommu uses a 2-level page table.
 * The first level is the "Directory Table" (DT).
 * The DT consists of 4096 4-byte Directory Table Entries (DTEs), each pointing
 * to a "Page Table".
 * The second level is the 256 Page Tables (PT).
 * Each PT consists of 256 4-byte Page Table Entries (PTEs), each pointing to
 * a 4 KB page of physical memory.
 *
 * The structure of the page table is as follows:
 *
 *                   DT
 * MMU_DTE_ADDR -> +-----+
 *                 |     |
 *                 +-----+     PT
 *                 | DTE | -> +-----+
 *                 +-----+    |     |     Memory
 *                 |     |    +-----+     Page
 *                 |     |    | PTE | -> +-----+
 *                 +-----+    +-----+    |     |
 *                            |     |    |     |
 *                            |     |    |     |
 *                            +-----+    |     |
 *                                       |     |
 *                                       |     |
 *                                       +-----+
 */

/*
 * Each DTE has a PT address and a valid bit:
 * +---------------------+-----------+-+
 * | PT address          | Reserved  |V|
 * +---------------------+-----------+-+
 *  31:10 - PT address (PTs always starts on a 1 KB boundary)
 *   9: 2 - Reserved
 *   1: 0 - 01 if PT @ PT address is valid
 */
#define SUN50I_DTE_PT_ADDRESS_MASK	GENMASK(31, 10)
#define SUN50I_DTE_PT_VALID		BIT(0)
#define SUN50I_DTE_PT_VALID_MASK	GENMASK(1, 0)

static inline phys_addr_t sun50i_dte_pt_address(u32 dte)
{
	return (phys_addr_t)dte & SUN50I_DTE_PT_ADDRESS_MASK;
}

static inline bool sun50i_dte_is_pt_valid(u32 dte)
{
	return (dte & SUN50I_DTE_PT_VALID_MASK) == SUN50I_DTE_PT_VALID;
}

static inline u32 sun50i_mk_dte(dma_addr_t pt_dma)
{
	return (pt_dma & SUN50I_DTE_PT_ADDRESS_MASK) | SUN50I_DTE_PT_VALID;
}

/*
 * Each PTE has a Page address, some flags and a valid bit:
 * +--------------+---+-------+---+---+---+
 * | Page address |Rsv| Flags |Rsv| V |Rsv|
 * +--------------+---+-------+---+---+---+
 *  31:12 - Page address (Pages always start on a 4 KB boundary)
 *  11: 8 - Reserved
 *   7: 4 - ACI - permission control index
 *   3: 2 - Reserved
 *      1 - 1 if Page @ Page address is valid
 *      0 - Reserved
 */
#define SUN50I_PTE_PAGE_ADDRESS_MASK	GENMASK(31, 12)
#define SUN50I_PTE_PAGE_VALID		BIT(1)

static inline phys_addr_t sun50i_pte_page_address(u32 pte)
{
	return (phys_addr_t)pte & SUN50I_PTE_PAGE_ADDRESS_MASK;
}

static inline bool sun50i_pte_is_page_valid(u32 pte)
{
	return pte & SUN50I_PTE_PAGE_VALID;
}

static u32 sun50i_mk_pte(phys_addr_t page, int prot)
{
	return (page & SUN50I_PTE_PAGE_ADDRESS_MASK) | SUN50I_PTE_PAGE_VALID;
}

static u32 sun50i_mk_pte_invalid(u32 pte)
{
	return pte & ~SUN50I_PTE_PAGE_VALID;
}

/*
 * sun50i-h6 iova (IOMMU Virtual Address) format
 *  31       20.19       12.11          0
 * +-----------+-----------+-------------+
 * | DTE index | PTE index | Page offset |
 * +-----------+-----------+-------------+
 *  31:20 - DTE index   - index of DTE in DT
 *  19:12 - PTE index   - index of PTE in PT @ DTE.pt_address
 *  11: 0 - Page offset - offset into page @ PTE.page_address
 */
#define SUN50I_IOVA_DTE_MASK	GENMASK(31, 20)
#define SUN50I_IOVA_DTE_SHIFT	20
#define SUN50I_IOVA_PTE_MASK	GENMASK(19, 12)
#define SUN50I_IOVA_PTE_SHIFT	12
#define SUN50I_IOVA_PAGE_MASK	GENMASK(11, 0)
#define SUN50I_IOVA_PAGE_SHIFT	0

static u32 sun50i_iova_dte_index(dma_addr_t iova)
{
	return (u32)(iova & SUN50I_IOVA_DTE_MASK) >> SUN50I_IOVA_DTE_SHIFT;
}

static u32 sun50i_iova_pte_index(dma_addr_t iova)
{
	return (u32)(iova & SUN50I_IOVA_PTE_MASK) >> SUN50I_IOVA_PTE_SHIFT;
}

static u32 sun50i_iova_page_offset(dma_addr_t iova)
{
	return (u32)(iova & SUN50I_IOVA_PAGE_MASK) >> SUN50I_IOVA_PAGE_SHIFT;
}

static struct sunxi_iommu_domain *to_sunxi_domain(struct iommu_domain *dom)
{
	return container_of(dom, struct sunxi_iommu_domain, domain);
}

static struct sunxi_iommu *to_sunxi_iommu(struct device *dev)
{
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);

	return fwspec ? fwspec->iommu_priv : NULL;
}

static void sun50i_iommu_hw_init(struct sunxi_iommu *iommu)
{
	struct iommu_domain *dom = iommu->domain;
	struct sunxi_iommu_domain *sunxi_domain = to_sunxi_domain(dom);

	regmap_write(iommu->regmap, SUN50I_IOMMU_TTB, sunxi_domain->dt_dma);
	regmap_write(iommu->regmap, SUN50I_IOMMU_TE, SUN50I_IOMMU_TLB_ENABLE);
	regmap_write(iommu->regmap, SUN50I_IOMMU_TP, SUN50I_IOMMU_TLB_PREFETCH);
	regmap_write(iommu->regmap, SUN50I_IOMMU_BYPASS,
		     SUN50I_IOMMU_BYPASS_VAL);
	regmap_write(iommu->regmap, SUN50I_IOMMU_EN, SUN50I_IOMMU_ENABLE);
	regmap_write(iommu->regmap, SUN50I_IOMMU_IE, SUN50I_IOMMU_INT_ENABLE);
}

static void sunxi_tlb_invalid(struct sunxi_iommu *iommu, dma_addr_t iova)
{
	int ret = 0;
	u32 val;

	regmap_write(iommu->regmap, SUN50I_IOMMU_TIA, iova);
	regmap_write(iommu->regmap, SUN50I_IOMMU_TIAM,
		     SUN50I_IOMMU_TLB_IVLD_ADDR_MASK);
	regmap_write(iommu->regmap, SUN50I_IOMMU_TIE,
		     SUN50I_IOMMU_TLB_IVLD_ENABLE);
	ret = regmap_read_poll_timeout(iommu->regmap, SUN50I_IOMMU_TIE, val,
				       !val, 0, SUNXI_MMU_POLL_TIMEOUT_US);
	if (ret)
		pr_err("TLB Invalid timed out\n");

	/* ptw cache invalid */
	regmap_write(iommu->regmap, SUN50I_IOMMU_PCIA, iova);
	regmap_write(iommu->regmap, SUN50I_IOMMU_PCIE,
		     SUN50I_IOMMU_PC_IVLD_ENABLE);
	ret = regmap_read_poll_timeout(iommu->regmap, SUN50I_IOMMU_PCIE, val,
				       !val, 0, SUNXI_MMU_POLL_TIMEOUT_US);
	if (ret)
		pr_err("PTW cache invalid timed out\n");
}

/* XXX: Is this the right way? */
void sunxi_zap_tlb(struct sunxi_iommu *iommu,
		   unsigned long iova, size_t size)
{
	sunxi_tlb_invalid(iommu, iova);
	sunxi_tlb_invalid(iommu, iova + SPAGE_SIZE);
	sunxi_tlb_invalid(iommu, iova + size);
	sunxi_tlb_invalid(iommu, iova + size + SPAGE_SIZE);
}

static u32 *sunxi_dte_get_page_table(struct sunxi_iommu_domain *sunxi_domain,
				     dma_addr_t iova)
{
	u32 dte_index, dte, *page_table, *dte_addr;
	phys_addr_t pt_phys;
	dma_addr_t pt_dma;

	assert_spin_locked(&sunxi_domain->dt_lock);

	dte_index = sun50i_iova_dte_index(iova);
	dte_addr = &sunxi_domain->dt[dte_index];
	dte = *dte_addr;
	if (sun50i_dte_is_pt_valid(dte))
		goto done;

	page_table = kmem_cache_zalloc(sunxi_domain->pt_kmem_cache,
				       GFP_ATOMIC | GFP_DMA32);
	if (!page_table)
		return ERR_PTR(-ENOMEM);

	pt_dma = dma_map_single(dma_dev, page_table, PT_SIZE, DMA_TO_DEVICE);
	if (dma_mapping_error(dma_dev, pt_dma)) {
		kmem_cache_free(sunxi_domain->pt_kmem_cache, page_table);
		return ERR_PTR(-ENOMEM);
	}
	kmemleak_ignore(page_table);

	dte = sun50i_mk_dte(pt_dma);
	*dte_addr = dte;

	sunxi_table_flush(pt_dma, NUM_PT_ENTRIES);
	sunxi_table_flush(sunxi_domain->dt_dma + dte_index * sizeof(u32), 1);
done:
	pt_phys = sun50i_dte_pt_address(dte);
	return (u32 *)phys_to_virt(pt_phys);
}

static size_t sun50i_iommu_unmap_iova(struct sunxi_iommu_domain *sunxi_domain,
				      u32 *pte_addr, dma_addr_t pte_dma,
				      size_t size)
{
	unsigned int pte_count;
	unsigned int pte_total = size / SPAGE_SIZE;

	assert_spin_locked(&sunxi_domain->dt_lock);

	for (pte_count = 0; pte_count < pte_total; pte_count++) {
		u32 pte = pte_addr[pte_count];

		if (!sun50i_pte_is_page_valid(pte))
			break;

		pte_addr[pte_count] = sun50i_mk_pte_invalid(pte);
	}

	sunxi_table_flush(pte_dma, pte_count);

	return pte_count * SPAGE_SIZE;
}

static int sun50i_iommu_map_iova(struct sunxi_iommu_domain *sunxi_domain,
				 u32 *pte_addr, dma_addr_t pte_dma,
				 dma_addr_t iova, phys_addr_t paddr,
				 size_t size, int prot)
{
	unsigned int pte_count;
	unsigned int pte_total = size / SPAGE_SIZE;
	phys_addr_t page_phys;

	assert_spin_locked(&sunxi_domain->dt_lock);

	for (pte_count = 0; pte_count < pte_total; pte_count++) {
		u32 pte = pte_addr[pte_count];

		if (sun50i_pte_is_page_valid(pte))
			goto unwind;

		pte_addr[pte_count] = sun50i_mk_pte(paddr, prot);

		paddr += SPAGE_SIZE;
	}

	sunxi_table_flush(pte_dma, pte_total);

	sunxi_zap_tlb(sunxi_domain->iommu, iova, size);

	return 0;
unwind:
	/* Unmap the range of iovas that we just mapped */
	sun50i_iommu_unmap_iova(sunxi_domain, pte_addr, pte_dma,
				pte_count * SPAGE_SIZE);

	iova += pte_count * SPAGE_SIZE;
	page_phys = sun50i_pte_page_address(pte_addr[pte_count]);
	pr_err("iova: %pad already mapped to %pa cannot remap to phys: %pa prot: %#x\n",
	       &iova, &page_phys, &paddr, prot);

	return -EADDRINUSE;
}

static int sun50i_iommu_map(struct iommu_domain *domain, unsigned long _iova,
			   phys_addr_t paddr, size_t size, int prot, gfp_t gfp)
{
	struct sunxi_iommu_domain *sunxi_domain = to_sunxi_domain(domain);
	unsigned long flags;
	dma_addr_t pte_dma, iova = (dma_addr_t)_iova;
	u32 *page_table, *pte_addr;
	u32 dte_index, pte_index;
	int ret;

	spin_lock_irqsave(&sunxi_domain->dt_lock, flags);

	/*
	 * pgsize_bitmap specifies iova sizes that fit in one page table
	 * (256 4-KiB pages = 1 MiB).
	 * So, size will always be 4096 <= size <= 1048576.
	 * Since iommu_map() guarantees that both iova and size will be
	 * aligned, we will always only be mapping from a single dte here.
	 */
	page_table = sunxi_dte_get_page_table(sunxi_domain, iova);
	if (IS_ERR(page_table)) {
		spin_unlock_irqrestore(&sunxi_domain->dt_lock, flags);
		return PTR_ERR(page_table);
	}

	dte_index = sunxi_domain->dt[sun50i_iova_dte_index(iova)];
	pte_index = sun50i_iova_pte_index(iova);
	pte_addr = &page_table[pte_index];
	pte_dma = sun50i_dte_pt_address(dte_index) + pte_index * sizeof(u32);
	ret = sun50i_iommu_map_iova(sunxi_domain, pte_addr, pte_dma, iova,
				    paddr, size, prot);
	spin_unlock_irqrestore(&sunxi_domain->dt_lock, flags);

	return ret;
}

static size_t sun50i_iommu_unmap(struct iommu_domain *domain,
				 unsigned long _iova, size_t size,
				 struct iommu_iotlb_gather *gather)
{
	struct sunxi_iommu_domain *sunxi_domain = to_sunxi_domain(domain);
	unsigned long flags;
	dma_addr_t pte_dma, iova = (dma_addr_t)_iova;
	phys_addr_t pt_phys;
	u32 dte, *pte_addr;
	size_t unmap_size;

	spin_lock_irqsave(&sunxi_domain->dt_lock, flags);

	dte = sunxi_domain->dt[sun50i_iova_dte_index(iova)];
	if (!sun50i_dte_is_pt_valid(dte)) {
		spin_unlock_irqrestore(&sunxi_domain->dt_lock, flags);
		return 0;
	}

	pt_phys = sun50i_dte_pt_address(dte);
	pte_addr = (u32 *)phys_to_virt(pt_phys) + sun50i_iova_pte_index(iova);
	pte_dma = pt_phys + sun50i_iova_pte_index(iova) * sizeof(u32);
	unmap_size = sun50i_iommu_unmap_iova(sunxi_domain, pte_addr,
					     pte_dma, size);

	spin_unlock_irqrestore(&sunxi_domain->dt_lock, flags);

	sunxi_zap_tlb(sunxi_domain->iommu, iova, unmap_size);

	return unmap_size;
}

static phys_addr_t sun50i_iommu_iova_to_phys(struct iommu_domain *domain,
					    dma_addr_t iova)
{
	struct sunxi_iommu_domain *sunxi_domain = to_sunxi_domain(domain);
	unsigned long flags;
	phys_addr_t pt_phys, phys = 0;
	u32 dte, pte, *page_table;

	spin_lock_irqsave(&sunxi_domain->dt_lock, flags);

	dte = sunxi_domain->dt[sun50i_iova_dte_index(iova)];
	if (!sun50i_dte_is_pt_valid(dte))
		goto out;

	pt_phys = sun50i_dte_pt_address(dte);
	page_table = (u32 *)phys_to_virt(pt_phys);
	pte = page_table[sun50i_iova_pte_index(iova)];
	if (!sun50i_pte_is_page_valid(pte))
		goto out;

	phys = sun50i_pte_page_address(pte) + sun50i_iova_page_offset(iova);
out:
	spin_unlock_irqrestore(&sunxi_domain->dt_lock, flags);

	return phys;
}

static struct iommu_domain *sun50i_iommu_domain_alloc(unsigned int type)
{
	struct sunxi_iommu_domain *sunxi_domain;

	if (type != IOMMU_DOMAIN_DMA && type != IOMMU_DOMAIN_UNMANAGED)
		return NULL;

	if (!dma_dev)
		return NULL;

	sunxi_domain = kzalloc(sizeof(*sunxi_domain), GFP_KERNEL);
	if (!sunxi_domain)
		return NULL;

	if (type == IOMMU_DOMAIN_DMA &&
	    iommu_get_dma_cookie(&sunxi_domain->domain))
		goto err_free_domain;

	sunxi_domain->dt = (u32 *)__get_free_pages(GFP_KERNEL | GFP_DMA32 |
						   __GFP_ZERO,
						   get_order(DT_SIZE));
	if (!sunxi_domain->dt)
		goto err_put_cookie;

	sunxi_domain->dt_dma = dma_map_single(dma_dev, sunxi_domain->dt,
					      DT_SIZE, DMA_TO_DEVICE);
	if (dma_mapping_error(dma_dev, sunxi_domain->dt_dma))
		goto err_free_dt;

	sunxi_domain->pt_kmem_cache = kmem_cache_create("sun50i-iommu-pt",
							PT_SIZE, PT_SIZE,
							0, NULL);
	if (!sunxi_domain->pt_kmem_cache)
		goto err_unmap;

	sunxi_table_flush(sunxi_domain->dt_dma, NUM_DT_ENTRIES);

	spin_lock_init(&sunxi_domain->dt_lock);

	sunxi_domain->domain.geometry.aperture_start = 0;
	sunxi_domain->domain.geometry.aperture_end = DMA_BIT_MASK(32);
	sunxi_domain->domain.geometry.force_aperture = true;

	return &sunxi_domain->domain;

err_unmap:
	dma_unmap_single(dma_dev, sunxi_domain->dt_dma,
			 SPAGE_SIZE, DMA_TO_DEVICE);
err_free_dt:
	free_page((unsigned long)sunxi_domain->dt);
err_put_cookie:
	if (type == IOMMU_DOMAIN_DMA)
		iommu_put_dma_cookie(&sunxi_domain->domain);
err_free_domain:
	kfree(sunxi_domain);

	return NULL;
}

static void sun50i_iommu_domain_free(struct iommu_domain *domain)
{
	struct sunxi_iommu_domain *sunxi_domain = to_sunxi_domain(domain);

	int i;

	for (i = 0; i < NUM_DT_ENTRIES; i++) {
		u32 dte = sunxi_domain->dt[i];

		if (sun50i_dte_is_pt_valid(dte)) {
			phys_addr_t pt_phys = sun50i_dte_pt_address(dte);
			u32 *page_table = phys_to_virt(pt_phys);

			dma_unmap_single(dma_dev, pt_phys,
					 PT_SIZE, DMA_TO_DEVICE);
			kmem_cache_free(sunxi_domain->pt_kmem_cache,
					page_table);
		}
	}
	kmem_cache_destroy(sunxi_domain->pt_kmem_cache);

	dma_unmap_single(dma_dev, sunxi_domain->dt_dma,
			 DT_SIZE, DMA_TO_DEVICE);
	free_page((unsigned long)sunxi_domain->dt);

	if (domain->type == IOMMU_DOMAIN_DMA)
		iommu_put_dma_cookie(&sunxi_domain->domain);
	kfree(sunxi_domain);
}

static void sun50i_iommu_config(struct sunxi_iommu *iommu,
				struct device *dev, bool enable)
{
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);
	u32 val = 0;

	if (!enable)
		val = BIT(fwspec->ids[0]);

	regmap_update_bits(iommu->regmap,
			   SUN50I_IOMMU_BYPASS,
			   BIT(fwspec->ids[0]),
			   val);
}

static void sun50i_iommu_detach_dev(struct iommu_domain *domain,
				   struct device *dev)
{
	struct sunxi_iommu *iommu = to_sunxi_iommu(dev);

	if (!iommu)
		return;

	sun50i_iommu_config(iommu, dev, false);
}

static int sun50i_iommu_attach_dev(struct iommu_domain *domain,
				  struct device *dev)
{
	struct sunxi_iommu *iommu = to_sunxi_iommu(dev);
	struct sunxi_iommu_domain *sunxi_domain = to_sunxi_domain(domain);

	if (!iommu)
		return -ENXIO;

	if (!iommu->domain) {
		iommu->domain = domain;
		sunxi_domain->iommu = iommu;
		sun50i_iommu_hw_init(iommu);
	}
	sun50i_iommu_config(iommu, dev, true);

	return 0;
}

static int sun50i_iommu_add_device(struct device *dev)
{
	struct sunxi_iommu *iommu = to_sunxi_iommu(dev);
	struct iommu_group *group;

	if (!iommu)
		return -ENODEV;

	iommu_device_link(&iommu->iommu, dev);

	group = iommu_group_get_for_dev(dev);
	if (IS_ERR(group))
		return PTR_ERR(group);
	iommu_group_put(group);

	return 0;
}

static void sun50i_iommu_remove_device(struct device *dev)
{
	struct sunxi_iommu *iommu = to_sunxi_iommu(dev);

	if (!iommu)
		return;

	iommu_device_unlink(&iommu->iommu, dev);

	iommu_group_remove_device(dev);
	iommu_fwspec_free(dev);
}

static struct iommu_group *sun50i_iommu_device_group(struct device *dev)
{
	struct sunxi_iommu *iommu = to_sunxi_iommu(dev);

	if (!iommu)
		return ERR_PTR(-ENODEV);

	/* All devices are in the same iommu-group */
	if (!iommu->group)
		iommu->group = iommu_group_alloc();
	else
		iommu_group_ref_get(iommu->group);

	return iommu->group;
}

static int sun50i_iommu_of_xlate(struct device *dev,
				struct of_phandle_args *args)
{
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);

	if (args->args_count != 1)
		return -EINVAL;

	if (!fwspec->iommu_priv) {
		struct platform_device *iommu_dev;

		iommu_dev = of_find_device_by_node(args->np);
		if (!iommu_dev)
			return -ENODEV;

		fwspec->iommu_priv = platform_get_drvdata(iommu_dev);
	}

	return iommu_fwspec_add_ids(dev, args->args, 1);
}

static struct iommu_ops sunxi_iommu_ops = {
	.of_xlate = sun50i_iommu_of_xlate,
	.add_device = sun50i_iommu_add_device,
	.remove_device = sun50i_iommu_remove_device,
	.device_group = sun50i_iommu_device_group,
	.domain_alloc = sun50i_iommu_domain_alloc,
	.domain_free = sun50i_iommu_domain_free,
	.attach_dev = sun50i_iommu_attach_dev,
	.detach_dev = sun50i_iommu_detach_dev,
	.map  = sun50i_iommu_map,
	.unmap = sun50i_iommu_unmap,
	.iova_to_phys = sun50i_iommu_iova_to_phys,
	.pgsize_bitmap = SUNXI_IOMMU_PGSIZE_BITMAP,
};

static irqreturn_t sunxi_iommu_irq(int irq, void *dev_id)
{
	struct sunxi_iommu *iommu = dev_id;
	u32 int_status, iova;

	regmap_read(iommu->regmap, SUN50I_IOMMU_IS, &int_status);
	if (int_status & L1_PAGETABLE_INVALID)
		regmap_read(iommu->regmap, SUN50I_IOMMU_IEA6, &iova);
	else
		regmap_read(iommu->regmap, SUN50I_IOMMU_IEA7, &iova);

	if (report_iommu_fault(iommu->domain, iommu->dev, iova, 0))
		dev_err_ratelimited(iommu->dev,
				    "fault type=0x%08x iova=0x%x\n",
				    int_status, iova);

	regmap_write(iommu->regmap, SUN50I_IOMMU_IC, int_status);

	sunxi_tlb_invalid(iommu, iova);

	return IRQ_HANDLED;
}

static const struct regmap_config config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.fast_io = true,
};

static int sunxi_iommu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sunxi_iommu *iommu;
	void __iomem *base;
	int ret, irq;

	iommu = devm_kzalloc(dev, sizeof(*iommu), GFP_KERNEL);
	if (!iommu)
		return -ENOMEM;

	platform_set_drvdata(pdev, iommu);
	iommu->dev = dev;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	iommu->regmap = devm_regmap_init_mmio(dev, base, &config);
	if (IS_ERR(iommu->regmap))
		return PTR_ERR(iommu->regmap);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	iommu->reset = devm_reset_control_get(dev, NULL);
	if (IS_ERR(iommu->reset))
		return PTR_ERR(iommu->reset);

	iommu->clk = devm_clk_get(dev, "iommu");
	if (IS_ERR(iommu->clk))
		return PTR_ERR(iommu->clk);

	ret = reset_control_deassert(iommu->reset);
	if (ret)
		return ret;

	ret = clk_prepare_enable(iommu->clk);
	if (ret)
		goto assert_reset;

	ret = iommu_device_sysfs_add(&iommu->iommu, dev, NULL, dev_name(dev));
	if (ret)
		goto clk_disable;

	iommu_device_set_ops(&iommu->iommu, &sunxi_iommu_ops);
	iommu_device_set_fwnode(&iommu->iommu, &dev->of_node->fwnode);

	ret = iommu_device_register(&iommu->iommu);
	if (ret)
		goto remove_sysfs;

	if (!dma_dev)
		dma_dev = dev;

	bus_set_iommu(&platform_bus_type, &sunxi_iommu_ops);

	ret = devm_request_irq(dev, irq, sunxi_iommu_irq, 0,
			       dev_name(dev), iommu);
	if (ret)
		goto unregister_iommu;

	return 0;

unregister_iommu:
	iommu_device_unregister(&iommu->iommu);
remove_sysfs:
	iommu_device_sysfs_remove(&iommu->iommu);
clk_disable:
	clk_disable_unprepare(iommu->clk);
assert_reset:
	reset_control_assert(iommu->reset);

	return ret;
}

static const struct of_device_id sunxi_iommu_of_match[] = {
	{ .compatible = "allwinner,sun50i-iommu", },
	{ /* sentinel */ },
};

static struct platform_driver sunxi_iommu_driver = {
	.probe		= sunxi_iommu_probe,
	.driver		= {
		.name		= "sun50i-iommu",
		.of_match_table = sunxi_iommu_of_match,
	}
};

static int __init sun50i_iommu_init(void)
{
	return  platform_driver_register(&sunxi_iommu_driver);
}
subsys_initcall(sun50i_iommu_init);
