/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2013 The FreeBSD Foundation
 * All rights reserved.
 *
 * This software was developed by Aleksandr Rybalko under sponsorship from the
 * FreeBSD Foundation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#ifndef _DEV_VT_HW_FB_VT_FB_H_
#define	_DEV_VT_HW_FB_VT_FB_H_
/* Generic framebuffer interface call vt_tft_attach to init VT(9) */
int vt_tft_attach(struct fb_info *info);
void vt_tft_resume(struct vt_device *vd);
void vt_tft_suspend(struct vt_device *vd);
int vt_tft_detach(struct fb_info *info);

vd_init_t		vt_tft_init;
vd_fini_t		vt_tft_fini;
vd_blank_t		vt_tft_blank;
vd_bitblt_text_t	vt_tft_bitblt_text;
vd_invalidate_text_t	vt_tft_invalidate_text;
vd_bitblt_bmp_t		vt_tft_bitblt_bitmap;
vd_drawrect_t		vt_tft_drawrect;
vd_setpixel_t		vt_tft_setpixel;
vd_postswitch_t		vt_tft_postswitch;
vd_fb_ioctl_t		vt_tft_ioctl;

#endif /* _DEV_VT_HW_FB_VT_FB_H_ */
