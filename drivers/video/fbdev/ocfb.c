/*
 * OpenCores VGA/LCD 2.0 core frame buffer driver
 *
 * Copyright (C) 2013 Stefan Kristiansson, stefan.kristiansson@saunalahti.fi
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/slab.h>

/* OCFB register defines */
#define OCFB_CTRL	0x000
#define OCFB_STAT	0x004
#define OCFB_HTIM	0x008
#define OCFB_VTIM	0x00c
#define OCFB_HVLEN	0x010
#define OCFB_VBARA	0x014
#define OCFB_PALETTE	0x800

#define OCFB_CTRL_VEN	0x00000001 /* Video Enable */
#define OCFB_CTRL_HIE	0x00000002 /* HSync Interrupt Enable */
#define OCFB_CTRL_PC	0x00000800 /* 8-bit Pseudo Color Enable*/
#define OCFB_CTRL_CD8	0x00000000 /* Color Depth 8 */
#define OCFB_CTRL_CD16	0x00000200 /* Color Depth 16 */
#define OCFB_CTRL_CD24	0x00000400 /* Color Depth 24 */
#define OCFB_CTRL_CD32	0x00000600 /* Color Depth 32 */
#define OCFB_CTRL_VBL1	0x00000000 /* Burst Length 1 */
#define OCFB_CTRL_VBL2	0x00000080 /* Burst Length 2 */
#define OCFB_CTRL_VBL4	0x00000100 /* Burst Length 4 */
#define OCFB_CTRL_VBL8	0x00000180 /* Burst Length 8 */

#define PALETTE_SIZE	256

#define OCFB_NAME	"OC VGA/LCD"

enum {
	OCFB_REGULAR,
	OCFB_EDL
};

static char *mode_option;
unsigned int vram_size = 1024 * 1280 * 2;

static const struct fb_videomode regular_modes[] = {
	/* 640x480 @ 60 Hz, 31.5 kHz hsync */
	{ NULL, 60, 640, 480, 39721, 40, 24, 32, 11, 96, 2,
	0, FB_VMODE_NONINTERLACED }
};

static const struct fb_videomode edl_modes[] = {
	/* 8 800x600-60 VESA */
	{ "800x600@60", 60, 800, 600, 25000, 88, 40, 23, 01, 128, 4,
	  FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	  FB_VMODE_NONINTERLACED, FB_MODE_IS_VESA },

	/* 13 1024x768-60 VESA */
	{ "1024x768@60", 60, 1024, 768, 15384, 160, 24, 29, 3, 136, 6,
	  0, FB_VMODE_NONINTERLACED, FB_MODE_IS_VESA },

	/* 1280x720 @ 60 Hz  -- checkme*/
	{ "1280x720@60", 60, 1280, 720, 13468, 220, 110, 20,  5,  40, 5,
	 1, FB_VMODE_NONINTERLACED},

	/* 17 1152x864-75 VESA */
	{ "1152x864@75", 75, 1152, 864, 9259, 256, 64, 32, 1, 128, 3,
	  FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	  FB_VMODE_NONINTERLACED, FB_MODE_IS_VESA },

	/* 20 1280x1024-60 VESA */
         { "1280x1024@60", 60, 1280, 1024, 9259, 248, 48, 38, 1, 112, 3,
           FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
           FB_VMODE_NONINTERLACED, FB_MODE_IS_VESA },
};

struct ocfb_dev {
	void __iomem *regs;
	/* flag indicating whether the regs are little endian accessed */
	int little_endian;
	/* Physical and virtual addresses of framebuffer */
	dma_addr_t fb_phys;
	void __iomem *fb_virt;
	int fbsize;
	u32 pseudo_palette[PALETTE_SIZE];
	int variant;
};

#ifndef MODULE
static int __init ocfb_setup(char *options)
{
	char *curr_opt;

	if (!options || !*options)
		return 0;

	while ((curr_opt = strsep(&options, ",")) != NULL) {
		if (!*curr_opt)
			continue;
		mode_option = curr_opt;
	}

	return 0;
}
#endif

static inline u32 ocfb_readreg(struct ocfb_dev *fbdev, loff_t offset)
{
	if (fbdev->little_endian)
		return ioread32(fbdev->regs + offset);
	else
		return ioread32be(fbdev->regs + offset);
}

static void ocfb_writereg(struct ocfb_dev *fbdev, loff_t offset, u32 data)
{
	if (fbdev->little_endian)
		iowrite32(data, fbdev->regs + offset);
	else
		iowrite32be(data, fbdev->regs + offset);
}

/* Calculates pixclk HW magic for EDL ocfb IP variant.
 * Francesco provided the following HW vs freq xlate table
 *  HW   freq
 *  0    40MHz
 *  1    65MHz
 *  2    74.25MHz
 *  3    108MHz
 */
static int ocfb_edl_pixclk(u32 pixclock)
{
	int i;
	/*  periods --    40MHz  65MHz 74.25MHz 108MHz */
	int ip_clks[] = { 25000, 15384, 13841, 9259 };
	for (i = 0; i < ARRAY_SIZE(ip_clks); i++) {
		if (abs(ip_clks[i] - pixclock) < 10)
			return i;
	}
	/* IP can't do this.. */
	return -1;
}

static int ocfb_init_fix(struct fb_info *info)
{
	struct fb_fix_screeninfo *fix = &info->fix;
	struct ocfb_dev *fbdev = (struct ocfb_dev *)info->par;

	strcpy(fix->id, OCFB_NAME);
	fix->accel = FB_ACCEL_NONE;
	fix->smem_len = fbdev->fbsize;
	fix->smem_start = fbdev->fb_phys;
	fix->type = FB_TYPE_PACKED_PIXELS;

	return 0;
}

static void ocfb_dealloc_fb(struct fb_info *info)
{
	struct device *dev = info->device;
	struct ocfb_dev *fbdev = (struct ocfb_dev *)info->par;

	if (fbdev->fb_virt) {
		dma_free_coherent(dev, fbdev->fbsize,
				fbdev->fb_virt, fbdev->fb_phys);
	}
	fbdev->fb_virt = NULL;
}

static int ocfb_setcolreg(unsigned regno, unsigned red, unsigned green,
			  unsigned blue, unsigned transp,
			  struct fb_info *info)
{
	struct ocfb_dev *fbdev = (struct ocfb_dev *)info->par;
	u32 color;

	if (regno >= info->cmap.len) {
		dev_err(info->device, "regno >= cmap.len\n");
		return 1;
	}

	if (info->var.grayscale) {
		/* grayscale = 0.30*R + 0.59*G + 0.11*B */
		red = green = blue = (red * 77 + green * 151 + blue * 28) >> 8;
	}

	red >>= (16 - info->var.red.length);
	green >>= (16 - info->var.green.length);
	blue >>= (16 - info->var.blue.length);
	transp >>= (16 - info->var.transp.length);

	if (info->var.bits_per_pixel == 8 && !info->var.grayscale) {
		regno <<= 2;
		color = (red << 16) | (green << 8) | blue;
		ocfb_writereg(fbdev, OCFB_PALETTE + regno, color);
	} else {
		((u32 *)(info->pseudo_palette))[regno] =
			(red << info->var.red.offset) |
			(green << info->var.green.offset) |
			(blue << info->var.blue.offset) |
			(transp << info->var.transp.offset);
	}

	return 0;
}

static int ocfb_set_par(struct fb_info *info)
{
	unsigned long bpp_config;
	int ret;
	u32 hlen;
	u32 vlen;
	struct ocfb_dev *fbdev = (struct ocfb_dev *)info->par;
	struct fb_var_screeninfo *var = &info->var;
	struct device *dev = info->device;
	u32 pix_clk = 0;

	/* Disable display */
	ocfb_writereg(fbdev, OCFB_CTRL, 0);

	/* Clear framebuffer */
	memset_io(fbdev->fb_virt, 0, fbdev->fbsize);

	info->fix.line_length = var->xres * var->bits_per_pixel / 8;

	if (var->bits_per_pixel == 8 && !var->grayscale)
		info->fix.visual = FB_VISUAL_PSEUDOCOLOR;
	else
		info->fix.visual = FB_VISUAL_TRUECOLOR;

	/* Horizontal timings */
	ocfb_writereg(fbdev, OCFB_HTIM, (var->hsync_len - 1) << 24 |
		      (var->left_margin - 1) << 16 | (var->xres - 1));

	/* Vertical timings */
	ocfb_writereg(fbdev, OCFB_VTIM, (var->vsync_len - 1) << 24 |
		      (var->upper_margin - 1) << 16 | (var->yres - 1));

	/* Total length of frame */
	hlen = var->left_margin + var->right_margin + var->hsync_len +
		var->xres;

	vlen = var->upper_margin + var->lower_margin + var->vsync_len +
		var->yres;

	ocfb_writereg(fbdev, OCFB_HVLEN, (hlen - 1) << 16 | (vlen - 1));

	switch (var->bits_per_pixel) {
	case 8:
		bpp_config = OCFB_CTRL_CD8;
		if (!var->grayscale)
			bpp_config |= OCFB_CTRL_PC;  /* enable palette */
		break;

	case 16:
		bpp_config = OCFB_CTRL_CD16;
		break;

	case 24:
		bpp_config = OCFB_CTRL_CD24;
		break;

	case 32:
		bpp_config = OCFB_CTRL_CD32;
		break;

	default:
		dev_err(dev, "Invalid bpp specified\n");
		return -EINVAL;
	}

	/* maximum (8) VBL (video memory burst length) */
	bpp_config |= OCFB_CTRL_VBL8;

	if (fbdev->variant == OCFB_EDL) {
		ret = ocfb_edl_pixclk(var->pixclock);

		if (ret < 0) {
			dev_err(dev, "Requested pixelclock %d is not supported", var->pixclock);
			return -EINVAL;
		} else {
			pix_clk = ret;
		}
	}

	/* Enable output and set pixclk */
	ocfb_writereg(fbdev, OCFB_CTRL, (OCFB_CTRL_VEN | bpp_config | pix_clk << 30));

	return 0;
}

static int ocfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	int req_ram;
	struct ocfb_dev *fbdev = (struct ocfb_dev *)info->par;

	/* we need our lines to be packed in vram.
	 * We try to fixup things (also other drivers seems to do things
	 * like this, even if this make X fail. The alternative is to
	 * return -EINVAL, that causes failure also).
	 */
	var->xres_virtual = var->xres;
	var->xoffset = 0;
	var->yoffset = 0;

	/* TODO: add check for regular variant */
	if ((fbdev->variant == OCFB_EDL) &&
		(ocfb_edl_pixclk(var->pixclock) < 0))
		return -EINVAL;

	req_ram = var->xres * var->yres_virtual * var->bits_per_pixel / 8;
	if (req_ram > fbdev->fbsize)
		return -EINVAL;

	switch (var->bits_per_pixel) {
	case 8:
		var->transp.offset = 0;
		var->transp.length = 0;
		var->red.offset = 0;
		var->red.length = 8;
		var->green.offset = 0;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;
		break;

	case 16:
		var->transp.offset = 0;
		var->transp.length = 0;
		var->red.offset = 11;
		var->red.length = 5;
		var->green.offset = 5;
		var->green.length = 6;
		var->blue.offset = 0;
		var->blue.length  = 5;
		break;

	case 24:
		var->transp.offset = 0;
		var->transp.length = 0;
		var->red.offset = 16;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;
		break;

	case 32:
		var->transp.offset = 24;
		var->transp.length = 8;
		var->red.offset = 16;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static struct fb_ops ocfb_ops = {
	.owner		= THIS_MODULE,
	.fb_setcolreg	= ocfb_setcolreg,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	.fb_set_par	= ocfb_set_par,
	.fb_check_var	= ocfb_check_var,
};

static struct of_device_id ocfb_match[] = {
	{
		.compatible = "opencores,ocfb",
		.data = (const void*)OCFB_REGULAR
	}, {
		.compatible = "opencores,ocfb-edl",
		.data = (const void*)OCFB_EDL
	},
	{},
};
MODULE_DEVICE_TABLE(of, ocfb_match);

static int ocfb_probe(struct platform_device *pdev)
{
	struct ocfb_dev *fbdev;
	struct resource *res;
	const struct fb_videomode *modes;
	int modes_len;
	struct device_node *np = pdev->dev.of_node;
	struct fb_info *info;
	int ret = 0;

	info = framebuffer_alloc(sizeof(*fbdev), &pdev->dev);
	if (!info)
		return -ENOMEM;

	fbdev = info->par;

	info->fbops = &ocfb_ops;
	info->par = fbdev;
	info->flags = FBINFO_DEFAULT;

	fbdev->variant = (int)of_match_node(ocfb_match, np)->data;
	fbdev->fb_virt = NULL;

	/* Request I/O resource */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "I/O resource request failed\n");
		return -ENXIO;
	}
	fbdev->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(fbdev->regs))
		return PTR_ERR(fbdev->regs);

	/* Allocate framebuffer memory */
	fbdev->fbsize = PAGE_ALIGN(vram_size);
	fbdev->fb_virt = dma_alloc_coherent(&pdev->dev, fbdev->fbsize,
					    &fbdev->fb_phys, GFP_KERNEL);
	if (!fbdev->fb_virt) {
		dev_err(&pdev->dev,
			"Frame buffer memory allocation failed\n");
		goto err_dma_free;
	}

	info->screen_base = fbdev->fb_virt;
	info->screen_size = fbdev->fbsize;
	info->pseudo_palette = fbdev->pseudo_palette;
	info->var.activate = FB_ACTIVATE_NOW;
	/* Register framebuffer address */
	fbdev->little_endian = 0;
	ocfb_writereg(fbdev, OCFB_VBARA, fbdev->fb_phys);

	/* Detect endianess */
	if (ocfb_readreg(fbdev, OCFB_VBARA) != fbdev->fb_phys) {
		fbdev->little_endian = 1;
		ocfb_writereg(fbdev, OCFB_VBARA, fbdev->fb_phys);
	}

	ocfb_init_fix(info);

	if (fbdev->little_endian)
		info->flags |= FBINFO_FOREIGN_ENDIAN;

	/* Allocate color map */
	ret = fb_alloc_cmap(&info->cmap, PALETTE_SIZE, 0);
	if (ret) {
		dev_err(&pdev->dev, "Color map allocation failed\n");
		goto err_dma_free;
	}

	if (fbdev->variant == OCFB_EDL) {
		modes = edl_modes;
		modes_len = ARRAY_SIZE(edl_modes);
	} else {
		/* regular */
		modes = regular_modes;
		modes_len = ARRAY_SIZE(regular_modes);
	}

	/* Video mode setup */
	if (!fb_find_mode(&info->var, info, mode_option,
				modes, modes_len, &modes[0], 16)) {
		dev_err(&pdev->dev, "No valid video modes found\n");
		return -EINVAL;
	}

	info->var.xres_virtual = info->var.xres;
	info->var.yres_virtual = info->var.yres;

	ret = ocfb_check_var(&info->var, info);
	if (ret) {
		return ret;
	}
	/* Setup and enable the framebuffer */
	ret = ocfb_set_par(info);
	if (ret) {
		return ret;
	}

	/* Register framebuffer */
	ret = register_framebuffer(info);
	if (ret) {
		dev_err(&pdev->dev, "Framebuffer registration failed\n");
		goto err_dealloc_cmap;
	}

	platform_set_drvdata(pdev, info);

	return 0;

err_dealloc_cmap:
	fb_dealloc_cmap(&info->cmap);

err_dma_free:
	ocfb_dealloc_fb(info);

	return ret;
}

static int ocfb_remove(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);
	struct ocfb_dev *fbdev = (struct ocfb_dev *)info->par;

	/* Disable display */
	ocfb_writereg(fbdev, OCFB_CTRL, 0);
	/* make sure DMA is stopped before freeing stuff */
	mb();

	unregister_framebuffer(info);

	fb_dealloc_cmap(&info->cmap);
	ocfb_dealloc_fb(info);

	platform_set_drvdata(pdev, NULL);
	framebuffer_release(info);

	return 0;
}

static struct platform_driver ocfb_driver = {
	.probe  = ocfb_probe,
	.remove	= ocfb_remove,
	.driver = {
		.name = "ocfb_fb",
		.of_match_table = ocfb_match,
	}
};

/*
 * Init and exit routines
 */
static int __init ocfb_init(void)
{
#ifndef MODULE
	char *option = NULL;

	if (fb_get_options("ocfb", &option))
		return -ENODEV;
	ocfb_setup(option);
#endif
	return platform_driver_register(&ocfb_driver);
}

static void __exit ocfb_exit(void)
{
	platform_driver_unregister(&ocfb_driver);
}

module_init(ocfb_init);
module_exit(ocfb_exit);

MODULE_AUTHOR("Stefan Kristiansson <stefan.kristiansson@saunalahti.fi>");
MODULE_DESCRIPTION("OpenCores VGA/LCD 2.0 frame buffer driver");
MODULE_LICENSE("GPL v2");
module_param(mode_option, charp, 0);
module_param(vram_size, uint, 0);
MODULE_PARM_DESC(mode_option, "Initial video mode ('<xres>x<yres>[-<bpp>][@refresh]')");
MODULE_PARM_DESC(mode_option, "Reserved VRAM");
