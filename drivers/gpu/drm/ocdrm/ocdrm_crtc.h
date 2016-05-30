/*
 * Open cores VGA/LCD 2.0 core DRM driver
 * Copyright (c) 2016 Istituto Italiano di Tecnologia
 * Electronic Design Lab.
 *
 * Author: Andrea Merello <andrea.merello@gmail.com>
 *
 * Based on the following drivers:
 *   - Analog Devices AXI HDMI DRM driver, which is
 *     Copyright 2012 Analog Devices Inc.
 *     Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 *   - Atmel atmel-hlcdc driver, which is
 *     Copyright (C) 2014 Traphandler
 *     Copyright (C) 2014 Free Electrons
 *     Author: Jean-Jacques Hiblot <jjhiblot@traphandler.com>
 *     Author: Boris BREZILLON <boris.brezillon@free-electrons.com>
 *
 *   - OpenCores VGA/LCD 2.0 core frame buffer driver
 *     Copyright (C) 2013 Stefan Kristiansson, stefan.kristiansson@saunalahti.fi
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _OCDRM_CRTC_H_
#define _OCDRM_CRTC_H_

#include "ocdrm_drv.h"

int ocdrm_crtc_create(struct ocdrm_priv *priv);

#endif
