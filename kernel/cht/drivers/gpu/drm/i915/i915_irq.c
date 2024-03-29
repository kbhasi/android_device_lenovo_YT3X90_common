/* i915_irq.c -- IRQ support for the I915 -*- linux-c -*-
 */
/*
 * Copyright 2003 Tungsten Graphics, Inc., Cedar Park, Texas.
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sub license, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * IN NO EVENT SHALL TUNGSTEN GRAPHICS AND/OR ITS SUPPLIERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/sysrq.h>
#include <linux/slab.h>
#include <linux/circ_buf.h>
#include <drm/drmP.h>
#include <drm/i915_drm.h>
#include "i915_drv.h"
#include "i915_trace.h"
#include <drm/i915_adf.h>
#include "intel_sync.h"
#include "intel_drv.h"
#include "intel_lrc_tdr.h"
#include "i915_scheduler.h"

#if defined (CONFIG_MIPI_DLP) || defined (CONFIG_HDMI_DLP)
#include <drm/intel_dual_display.h>
#endif

static const u32 hpd_ibx[] = {
	[HPD_CRT] = SDE_CRT_HOTPLUG,
	[HPD_SDVO_B] = SDE_SDVOB_HOTPLUG,
	[HPD_PORT_B] = SDE_PORTB_HOTPLUG,
	[HPD_PORT_C] = SDE_PORTC_HOTPLUG,
	[HPD_PORT_D] = SDE_PORTD_HOTPLUG
};

static const u32 hpd_cpt[] = {
	[HPD_CRT] = SDE_CRT_HOTPLUG_CPT,
	[HPD_SDVO_B] = SDE_SDVOB_HOTPLUG_CPT,
	[HPD_PORT_B] = SDE_PORTB_HOTPLUG_CPT,
	[HPD_PORT_C] = SDE_PORTC_HOTPLUG_CPT,
	[HPD_PORT_D] = SDE_PORTD_HOTPLUG_CPT
};

static const u32 hpd_mask_i915[] = {
	[HPD_CRT] = CRT_HOTPLUG_INT_EN,
	[HPD_SDVO_B] = SDVOB_HOTPLUG_INT_EN,
	[HPD_SDVO_C] = SDVOC_HOTPLUG_INT_EN,
	[HPD_PORT_B] = PORTB_HOTPLUG_INT_EN,
	[HPD_PORT_C] = PORTC_HOTPLUG_INT_EN,
	[HPD_PORT_D] = PORTD_HOTPLUG_INT_EN
};

static const u32 hpd_status_g4x[] = {
	[HPD_CRT] = CRT_HOTPLUG_INT_STATUS,
	[HPD_SDVO_B] = SDVOB_HOTPLUG_INT_STATUS_G4X,
	[HPD_SDVO_C] = SDVOC_HOTPLUG_INT_STATUS_G4X,
	[HPD_PORT_B] = PORTB_HOTPLUG_INT_STATUS,
	[HPD_PORT_C] = PORTC_HOTPLUG_INT_STATUS,
	[HPD_PORT_D] = PORTD_HOTPLUG_INT_STATUS
};

static const u32 hpd_status_i915[] = { /* i915 and valleyview are the same */
	[HPD_CRT] = CRT_HOTPLUG_INT_STATUS,
	[HPD_SDVO_B] = SDVOB_HOTPLUG_INT_STATUS_I915,
	[HPD_SDVO_C] = SDVOC_HOTPLUG_INT_STATUS_I915,
	[HPD_PORT_B] = PORTB_HOTPLUG_INT_STATUS,
	[HPD_PORT_C] = PORTC_HOTPLUG_INT_STATUS,
	[HPD_PORT_D] = PORTD_HOTPLUG_INT_STATUS
};

/* IIR can theoretically queue up two events. Be paranoid. */
#define GEN8_IRQ_RESET_NDX(type, which) do { \
	I915_WRITE(GEN8_##type##_IMR(which), 0xffffffff); \
	POSTING_READ(GEN8_##type##_IMR(which)); \
	I915_WRITE(GEN8_##type##_IER(which), 0); \
	I915_WRITE(GEN8_##type##_IIR(which), 0xffffffff); \
	POSTING_READ(GEN8_##type##_IIR(which)); \
	I915_WRITE(GEN8_##type##_IIR(which), 0xffffffff); \
	POSTING_READ(GEN8_##type##_IIR(which)); \
} while (0)

#define GEN5_IRQ_RESET(type) do { \
	I915_WRITE(type##IMR, 0xffffffff); \
	POSTING_READ(type##IMR); \
	I915_WRITE(type##IER, 0); \
	I915_WRITE(type##IIR, 0xffffffff); \
	POSTING_READ(type##IIR); \
	I915_WRITE(type##IIR, 0xffffffff); \
	POSTING_READ(type##IIR); \
} while (0)

/*
 * We should clear IMR at preinstall/uninstall, and just check at postinstall.
 */
#define GEN5_ASSERT_IIR_IS_ZERO(reg) do { \
	u32 val = I915_READ(reg); \
	if (val) { \
		WARN(1, "Interrupt register 0x%x is not zero: 0x%08x\n", \
		     (reg), val); \
		I915_WRITE((reg), 0xffffffff); \
		POSTING_READ(reg); \
		I915_WRITE((reg), 0xffffffff); \
		POSTING_READ(reg); \
	} \
} while (0)

#define GEN8_IRQ_INIT_NDX(type, which, imr_val, ier_val) do { \
	GEN5_ASSERT_IIR_IS_ZERO(GEN8_##type##_IIR(which)); \
	I915_WRITE(GEN8_##type##_IMR(which), (imr_val)); \
	I915_WRITE(GEN8_##type##_IER(which), (ier_val)); \
	POSTING_READ(GEN8_##type##_IER(which)); \
} while (0)

#define GEN5_IRQ_INIT(type, imr_val, ier_val) do { \
	GEN5_ASSERT_IIR_IS_ZERO(type##IIR); \
	I915_WRITE(type##IMR, (imr_val)); \
	I915_WRITE(type##IER, (ier_val)); \
	POSTING_READ(type##IER); \
} while (0)

/* For display hotplug interrupt */
static void
ironlake_enable_display_irq(struct drm_i915_private *dev_priv, u32 mask)
{
	assert_spin_locked(&dev_priv->irq_lock);

	if (WARN_ON(dev_priv->pm.irqs_disabled))
		return;

	if ((dev_priv->irq_mask & mask) != 0) {
		dev_priv->irq_mask &= ~mask;
		I915_WRITE(DEIMR, dev_priv->irq_mask);
		POSTING_READ(DEIMR);
	}
}

static void
ironlake_disable_display_irq(struct drm_i915_private *dev_priv, u32 mask)
{
	assert_spin_locked(&dev_priv->irq_lock);

	if (WARN_ON(dev_priv->pm.irqs_disabled))
		return;

	if ((dev_priv->irq_mask & mask) != mask) {
		dev_priv->irq_mask |= mask;
		I915_WRITE(DEIMR, dev_priv->irq_mask);
		POSTING_READ(DEIMR);
	}
}

/**
 * ilk_update_gt_irq - update GTIMR
 * @dev_priv: driver private
 * @interrupt_mask: mask of interrupt bits to update
 * @enabled_irq_mask: mask of interrupt bits to enable
 */
static void ilk_update_gt_irq(struct drm_i915_private *dev_priv,
			      uint32_t interrupt_mask,
			      uint32_t enabled_irq_mask)
{
	assert_spin_locked(&dev_priv->irq_lock);

	if (WARN_ON(dev_priv->pm.irqs_disabled))
		return;

	dev_priv->gt_irq_mask &= ~interrupt_mask;
	dev_priv->gt_irq_mask |= (~enabled_irq_mask & interrupt_mask);
	I915_WRITE(GTIMR, dev_priv->gt_irq_mask);
	POSTING_READ(GTIMR);
}

void ilk_enable_gt_irq(struct drm_i915_private *dev_priv, uint32_t mask)
{
	ilk_update_gt_irq(dev_priv, mask, mask);
}

void ilk_disable_gt_irq(struct drm_i915_private *dev_priv, uint32_t mask)
{
	ilk_update_gt_irq(dev_priv, mask, 0);
}

/**
  * snb_update_pm_irq - update GEN6_PMIMR
  * @dev_priv: driver private
  * @interrupt_mask: mask of interrupt bits to update
  * @enabled_irq_mask: mask of interrupt bits to enable
  */
static void snb_update_pm_irq(struct drm_i915_private *dev_priv,
			      uint32_t interrupt_mask,
			      uint32_t enabled_irq_mask)
{
	uint32_t new_val;

	assert_spin_locked(&dev_priv->irq_lock);

	if (WARN_ON(dev_priv->pm.irqs_disabled))
		return;

	new_val = dev_priv->pm_irq_mask;
	new_val &= ~interrupt_mask;
	new_val |= (~enabled_irq_mask & interrupt_mask);

	if (new_val != dev_priv->pm_irq_mask) {
		dev_priv->pm_irq_mask = new_val;
		I915_WRITE(GEN6_PMIMR, dev_priv->pm_irq_mask);
		POSTING_READ(GEN6_PMIMR);
	}
}

void snb_enable_pm_irq(struct drm_i915_private *dev_priv, uint32_t mask)
{
	snb_update_pm_irq(dev_priv, mask, mask);
}

void snb_disable_pm_irq(struct drm_i915_private *dev_priv, uint32_t mask)
{
	snb_update_pm_irq(dev_priv, mask, 0);
}

static bool ivb_can_enable_err_int(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *crtc;
	enum pipe pipe;

	assert_spin_locked(&dev_priv->irq_lock);

	for_each_pipe(pipe) {
		crtc = to_intel_crtc(dev_priv->pipe_to_crtc_mapping[pipe]);

		if (crtc->cpu_fifo_underrun_disabled)
			return false;
	}

	return true;
}

/**
  * bdw_update_pm_irq - update GT interrupt 2
  * @dev_priv: driver private
  * @interrupt_mask: mask of interrupt bits to update
  * @enabled_irq_mask: mask of interrupt bits to enable
  *
  * Copied from the snb function, updated with relevant register offsets
  */
static void bdw_update_pm_irq(struct drm_i915_private *dev_priv,
			      uint32_t interrupt_mask,
			      uint32_t enabled_irq_mask)
{
	uint32_t new_val;

	assert_spin_locked(&dev_priv->irq_lock);

	if (WARN_ON(dev_priv->pm.irqs_disabled))
		return;

	new_val = dev_priv->pm_irq_mask;
	new_val &= ~interrupt_mask;
	new_val |= (~enabled_irq_mask & interrupt_mask);

	if (new_val != dev_priv->pm_irq_mask) {
		dev_priv->pm_irq_mask = new_val;
		I915_WRITE(GEN8_GT_IMR(2), dev_priv->pm_irq_mask);
		POSTING_READ(GEN8_GT_IMR(2));
	}
}

void bdw_enable_pm_irq(struct drm_i915_private *dev_priv, uint32_t mask)
{
	bdw_update_pm_irq(dev_priv, mask, mask);
}

void bdw_disable_pm_irq(struct drm_i915_private *dev_priv, uint32_t mask)
{
	bdw_update_pm_irq(dev_priv, mask, 0);
}

static bool cpt_can_enable_serr_int(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	enum pipe pipe;
	struct intel_crtc *crtc;

	assert_spin_locked(&dev_priv->irq_lock);

	for_each_pipe(pipe) {
		crtc = to_intel_crtc(dev_priv->pipe_to_crtc_mapping[pipe]);

		if (crtc->pch_fifo_underrun_disabled)
			return false;
	}

	return true;
}

void i9xx_check_fifo_underruns(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *crtc;
	unsigned long flags;

	spin_lock_irqsave(&dev_priv->irq_lock, flags);

	for_each_intel_crtc(dev, crtc) {
		u32 reg = PIPESTAT(crtc->pipe);
		u32 pipestat;

		if (crtc->cpu_fifo_underrun_disabled)
			continue;

		pipestat = I915_READ(reg) & 0xffff0000;
		if ((pipestat & PIPE_FIFO_UNDERRUN_STATUS) == 0)
			continue;

		I915_WRITE(reg, pipestat | PIPE_FIFO_UNDERRUN_STATUS);
		POSTING_READ(reg);

		DRM_ERROR("pipe %c underrun\n", pipe_name(crtc->pipe));
	}

	spin_unlock_irqrestore(&dev_priv->irq_lock, flags);
}

static void i9xx_set_fifo_underrun_reporting(struct drm_device *dev,
					     enum pipe pipe,
					     bool enable, bool old)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 reg = PIPESTAT(pipe);
	u32 pipestat = I915_READ(reg) & 0xffff0000;

	assert_spin_locked(&dev_priv->irq_lock);

	if (enable) {
		I915_WRITE(reg, pipestat | PIPE_FIFO_UNDERRUN_STATUS);
		POSTING_READ(reg);
	} else {
		if (old && pipestat & PIPE_FIFO_UNDERRUN_STATUS)
			DRM_ERROR("pipe %c underrun\n", pipe_name(pipe));
	}
}

static void ironlake_set_fifo_underrun_reporting(struct drm_device *dev,
						 enum pipe pipe, bool enable)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	uint32_t bit = (pipe == PIPE_A) ? DE_PIPEA_FIFO_UNDERRUN :
					  DE_PIPEB_FIFO_UNDERRUN;

	if (enable)
		ironlake_enable_display_irq(dev_priv, bit);
	else
		ironlake_disable_display_irq(dev_priv, bit);
}

static void ivybridge_set_fifo_underrun_reporting(struct drm_device *dev,
						  enum pipe pipe,
						  bool enable, bool old)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	if (enable) {
		I915_WRITE(GEN7_ERR_INT, ERR_INT_FIFO_UNDERRUN(pipe));

		if (!ivb_can_enable_err_int(dev))
			return;

		ironlake_enable_display_irq(dev_priv, DE_ERR_INT_IVB);
	} else {
		ironlake_disable_display_irq(dev_priv, DE_ERR_INT_IVB);

		if (old &&
		    I915_READ(GEN7_ERR_INT) & ERR_INT_FIFO_UNDERRUN(pipe)) {
			DRM_ERROR("uncleared fifo underrun on pipe %c\n",
				  pipe_name(pipe));
		}
	}
}

static void broadwell_set_fifo_underrun_reporting(struct drm_device *dev,
						  enum pipe pipe, bool enable)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	assert_spin_locked(&dev_priv->irq_lock);

	if (enable)
		dev_priv->de_irq_mask[pipe] &= ~GEN8_PIPE_FIFO_UNDERRUN;
	else
		dev_priv->de_irq_mask[pipe] |= GEN8_PIPE_FIFO_UNDERRUN;
	I915_WRITE(GEN8_DE_PIPE_IMR(pipe), dev_priv->de_irq_mask[pipe]);
	POSTING_READ(GEN8_DE_PIPE_IMR(pipe));
}

/**
 * ibx_display_interrupt_update - update SDEIMR
 * @dev_priv: driver private
 * @interrupt_mask: mask of interrupt bits to update
 * @enabled_irq_mask: mask of interrupt bits to enable
 */
static void ibx_display_interrupt_update(struct drm_i915_private *dev_priv,
					 uint32_t interrupt_mask,
					 uint32_t enabled_irq_mask)
{
	uint32_t sdeimr = I915_READ(SDEIMR);
	sdeimr &= ~interrupt_mask;
	sdeimr |= (~enabled_irq_mask & interrupt_mask);

	assert_spin_locked(&dev_priv->irq_lock);

	if (WARN_ON(dev_priv->pm.irqs_disabled))
		return;

	I915_WRITE(SDEIMR, sdeimr);
	POSTING_READ(SDEIMR);
}
#define ibx_enable_display_interrupt(dev_priv, bits) \
	ibx_display_interrupt_update((dev_priv), (bits), (bits))
#define ibx_disable_display_interrupt(dev_priv, bits) \
	ibx_display_interrupt_update((dev_priv), (bits), 0)

static void ibx_set_fifo_underrun_reporting(struct drm_device *dev,
					    enum transcoder pch_transcoder,
					    bool enable)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	uint32_t bit = (pch_transcoder == TRANSCODER_A) ?
		       SDE_TRANSA_FIFO_UNDER : SDE_TRANSB_FIFO_UNDER;

	if (enable)
		ibx_enable_display_interrupt(dev_priv, bit);
	else
		ibx_disable_display_interrupt(dev_priv, bit);
}

static void cpt_set_fifo_underrun_reporting(struct drm_device *dev,
					    enum transcoder pch_transcoder,
					    bool enable, bool old)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (enable) {
		I915_WRITE(SERR_INT,
			   SERR_INT_TRANS_FIFO_UNDERRUN(pch_transcoder));

		if (!cpt_can_enable_serr_int(dev))
			return;

		ibx_enable_display_interrupt(dev_priv, SDE_ERROR_CPT);
	} else {
		ibx_disable_display_interrupt(dev_priv, SDE_ERROR_CPT);

		if (old && I915_READ(SERR_INT) &
		    SERR_INT_TRANS_FIFO_UNDERRUN(pch_transcoder)) {
			DRM_ERROR("uncleared pch fifo underrun on pch transcoder %c\n",
				  transcoder_name(pch_transcoder));
		}
	}
}

/**
 * intel_set_cpu_fifo_underrun_reporting - enable/disable FIFO underrun messages
 * @dev: drm device
 * @pipe: pipe
 * @enable: true if we want to report FIFO underrun errors, false otherwise
 *
 * This function makes us disable or enable CPU fifo underruns for a specific
 * pipe. Notice that on some Gens (e.g. IVB, HSW), disabling FIFO underrun
 * reporting for one pipe may also disable all the other CPU error interruts for
 * the other pipes, due to the fact that there's just one interrupt mask/enable
 * bit for all the pipes.
 *
 * Returns the previous state of underrun reporting.
 */
static bool __intel_set_cpu_fifo_underrun_reporting(struct drm_device *dev,
						    enum pipe pipe, bool enable)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_crtc *crtc = dev_priv->pipe_to_crtc_mapping[pipe];
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	bool old;

	assert_spin_locked(&dev_priv->irq_lock);

	old = !intel_crtc->cpu_fifo_underrun_disabled;
	intel_crtc->cpu_fifo_underrun_disabled = !enable;

	if (INTEL_INFO(dev)->gen < 5 || IS_VALLEYVIEW(dev))
		i9xx_set_fifo_underrun_reporting(dev, pipe, enable, old);
	else if (IS_GEN5(dev) || IS_GEN6(dev))
		ironlake_set_fifo_underrun_reporting(dev, pipe, enable);
	else if (IS_GEN7(dev))
		ivybridge_set_fifo_underrun_reporting(dev, pipe, enable, old);
	else if (IS_GEN8(dev))
		broadwell_set_fifo_underrun_reporting(dev, pipe, enable);

	return old;
}

bool intel_set_cpu_fifo_underrun_reporting(struct drm_device *dev,
					   enum pipe pipe, bool enable)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long flags;
	bool ret;

	spin_lock_irqsave(&dev_priv->irq_lock, flags);
	ret = __intel_set_cpu_fifo_underrun_reporting(dev, pipe, enable);
	spin_unlock_irqrestore(&dev_priv->irq_lock, flags);

	return ret;
}

static bool __cpu_fifo_underrun_reporting_enabled(struct drm_device *dev,
						  enum pipe pipe)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_crtc *crtc = dev_priv->pipe_to_crtc_mapping[pipe];
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);

	return !intel_crtc->cpu_fifo_underrun_disabled;
}

/**
 * intel_set_pch_fifo_underrun_reporting - enable/disable FIFO underrun messages
 * @dev: drm device
 * @pch_transcoder: the PCH transcoder (same as pipe on IVB and older)
 * @enable: true if we want to report FIFO underrun errors, false otherwise
 *
 * This function makes us disable or enable PCH fifo underruns for a specific
 * PCH transcoder. Notice that on some PCHs (e.g. CPT/PPT), disabling FIFO
 * underrun reporting for one transcoder may also disable all the other PCH
 * error interruts for the other transcoders, due to the fact that there's just
 * one interrupt mask/enable bit for all the transcoders.
 *
 * Returns the previous state of underrun reporting.
 */
bool intel_set_pch_fifo_underrun_reporting(struct drm_device *dev,
					   enum transcoder pch_transcoder,
					   bool enable)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_crtc *crtc = dev_priv->pipe_to_crtc_mapping[pch_transcoder];
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	unsigned long flags;
	bool old;

	/*
	 * NOTE: Pre-LPT has a fixed cpu pipe -> pch transcoder mapping, but LPT
	 * has only one pch transcoder A that all pipes can use. To avoid racy
	 * pch transcoder -> pipe lookups from interrupt code simply store the
	 * underrun statistics in crtc A. Since we never expose this anywhere
	 * nor use it outside of the fifo underrun code here using the "wrong"
	 * crtc on LPT won't cause issues.
	 */

	spin_lock_irqsave(&dev_priv->irq_lock, flags);

	old = !intel_crtc->pch_fifo_underrun_disabled;
	intel_crtc->pch_fifo_underrun_disabled = !enable;

	if (HAS_PCH_IBX(dev))
		ibx_set_fifo_underrun_reporting(dev, pch_transcoder, enable);
	else
		cpt_set_fifo_underrun_reporting(dev, pch_transcoder, enable, old);

	spin_unlock_irqrestore(&dev_priv->irq_lock, flags);
	return old;
}


static void
__i915_enable_pipestat(struct drm_i915_private *dev_priv, enum pipe pipe,
		       u32 enable_mask, u32 status_mask)
{
	u32 reg = PIPESTAT(pipe);
	u32 pipestat = I915_READ(reg) & PIPESTAT_INT_ENABLE_MASK;
	u32 vlv_psr_status[I915_MAX_PIPES] = {PIPE_A_PSR_STATUS_VLV,
					PIPE_B_PSR_STATUS_VLV, 0};

	assert_spin_locked(&dev_priv->irq_lock);

	if (WARN_ONCE(enable_mask & ~PIPESTAT_INT_ENABLE_MASK ||
		      status_mask & ~PIPESTAT_INT_STATUS_MASK,
		      "pipe %c: enable_mask=0x%x, status_mask=0x%x\n",
		      pipe_name(pipe), enable_mask, status_mask))
		return;

	if ((pipestat & enable_mask) == enable_mask)
		return;

	if (IS_CHERRYVIEW(dev_priv->dev))
		status_mask |= vlv_psr_status[pipe];

	dev_priv->pipestat_irq_mask[pipe] |= status_mask;

	/* Enable the interrupt, clear any pending status */
	pipestat |= enable_mask | status_mask;
	I915_WRITE(reg, pipestat);
	POSTING_READ(reg);
}

static void
__i915_disable_pipestat(struct drm_i915_private *dev_priv, enum pipe pipe,
		        u32 enable_mask, u32 status_mask)
{
	u32 reg = PIPESTAT(pipe);
	u32 pipestat = I915_READ(reg) & PIPESTAT_INT_ENABLE_MASK;

	assert_spin_locked(&dev_priv->irq_lock);

	if (WARN_ONCE(enable_mask & ~PIPESTAT_INT_ENABLE_MASK ||
		      status_mask & ~PIPESTAT_INT_STATUS_MASK,
		      "pipe %c: enable_mask=0x%x, status_mask=0x%x\n",
		      pipe_name(pipe), enable_mask, status_mask))
		return;

	if ((pipestat & enable_mask) == 0)
		return;

	dev_priv->pipestat_irq_mask[pipe] &= ~status_mask;

	pipestat &= ~enable_mask;
	I915_WRITE(reg, pipestat);
	POSTING_READ(reg);
}

static u32 vlv_get_pipestat_enable_mask(struct drm_device *dev, u32 status_mask)
{
	u32 enable_mask = status_mask << 16;

	/*
	 * On pipe A we don't support the PSR interrupt yet,
	 * on pipe B and C the same bit MBZ.
	 */
	if (WARN_ON_ONCE(status_mask & PIPE_A_PSR_STATUS_VLV))
		return 0;
	/*
	 * On pipe B and C we don't support the PSR interrupt yet, on pipe
	 * A the same bit is for perf counters which we don't use either.
	 */
	if (WARN_ON_ONCE(status_mask & PIPE_B_PSR_STATUS_VLV))
		return 0;

	enable_mask &= ~(PIPE_FIFO_UNDERRUN_STATUS |
			 SPRITE0_FLIP_DONE_INT_EN_VLV |
			 SPRITE1_FLIP_DONE_INT_EN_VLV);
	if (status_mask & SPRITE0_FLIP_DONE_INT_STATUS_VLV)
		enable_mask |= SPRITE0_FLIP_DONE_INT_EN_VLV;
	if (status_mask & SPRITE1_FLIP_DONE_INT_STATUS_VLV)
		enable_mask |= SPRITE1_FLIP_DONE_INT_EN_VLV;

	return enable_mask;
}

void
i915_enable_pipestat(struct drm_i915_private *dev_priv, enum pipe pipe,
		     u32 status_mask)
{
	u32 enable_mask;

	if (IS_VALLEYVIEW(dev_priv->dev))
		enable_mask = vlv_get_pipestat_enable_mask(dev_priv->dev,
							   status_mask);
	else
		enable_mask = status_mask << 16;
	__i915_enable_pipestat(dev_priv, pipe, enable_mask, status_mask);
}

void
i915_disable_pipestat(struct drm_i915_private *dev_priv, enum pipe pipe,
		      u32 status_mask)
{
	u32 enable_mask;

	if (IS_VALLEYVIEW(dev_priv->dev))
		enable_mask = vlv_get_pipestat_enable_mask(dev_priv->dev,
							   status_mask);
	else
		enable_mask = status_mask << 16;
	__i915_disable_pipestat(dev_priv, pipe, enable_mask, status_mask);
}

#ifdef CONFIG_SUPPORT_LPDMA_HDMI_AUDIO
void
i915_enable_lpe_pipestat(struct drm_i915_private *dev_priv, int pipe)
{
	u32 mask, reg;
	mask = dev_priv->hdmi_audio_interrupt_mask;
	mask |= (I915_HDMI_AUDIO_UNDERRUN | I915_HDMI_AUDIO_BUFFER_DONE);

	/* Enable the interrupt, clear any pending status */
	if (IS_CHERRYVIEW(dev_priv->dev)) {
		reg = I915_LPE_AUDIO_HDMI_STATUS(pipe);
		I915_WRITE(reg, mask);
		POSTING_READ(reg);
	} else {
		I915_WRITE(I915_LPE_AUDIO_HDMI_STATUS_B, mask);
		POSTING_READ(I915_LPE_AUDIO_HDMI_STATUS_B);
	}
}

void
i915_disable_lpe_pipestat(struct drm_i915_private *dev_priv, int pipe)
{
	u32 mask, reg;
	mask = dev_priv->hdmi_audio_interrupt_mask;
	mask |= (I915_HDMI_AUDIO_UNDERRUN | I915_HDMI_AUDIO_BUFFER_DONE);

	/* Disable the interrupt, clear any pending status */
	if (IS_CHERRYVIEW(dev_priv->dev)) {
		reg = I915_LPE_AUDIO_HDMI_STATUS(pipe);
		I915_WRITE(reg, mask);
		POSTING_READ(reg);
	} else {
		I915_WRITE(I915_LPE_AUDIO_HDMI_STATUS_B, mask);
		POSTING_READ(I915_LPE_AUDIO_HDMI_STATUS_B);
	}
}
#endif

/**
 * i915_enable_asle_pipestat - enable ASLE pipestat for OpRegion
 */
static void i915_enable_asle_pipestat(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long irqflags;

	if (!dev_priv->opregion.asle || !IS_MOBILE(dev))
		return;

	spin_lock_irqsave(&dev_priv->irq_lock, irqflags);

	i915_enable_pipestat(dev_priv, PIPE_B, PIPE_LEGACY_BLC_EVENT_STATUS);
	if (INTEL_INFO(dev)->gen >= 4)
		i915_enable_pipestat(dev_priv, PIPE_A,
				     PIPE_LEGACY_BLC_EVENT_STATUS);

	spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);
}

/**
 * i915_pipe_enabled - check if a pipe is enabled
 * @dev: DRM device
 * @pipe: pipe to check
 *
 * Reading certain registers when the pipe is disabled can hang the chip.
 * Use this routine to make sure the PLL is running and the pipe is active
 * before reading such registers if unsure.
 */
static int
i915_pipe_enabled(struct drm_device *dev, int pipe)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (drm_core_check_feature(dev, DRIVER_MODESET)) {
		/* Locking is horribly broken here, but whatever. */
		struct drm_crtc *crtc = dev_priv->pipe_to_crtc_mapping[pipe];
		struct intel_crtc *intel_crtc = to_intel_crtc(crtc);

		return intel_crtc->active;
	} else {
		return I915_READ(PIPECONF(pipe)) & PIPECONF_ENABLE;
	}
}

/*
 * This timing diagram depicts the video signal in and
 * around the vertical blanking period.
 *
 * Assumptions about the fictitious mode used in this example:
 *  vblank_start >= 3
 *  vsync_start = vblank_start + 1
 *  vsync_end = vblank_start + 2
 *  vtotal = vblank_start + 3
 *
 *           start of vblank:
 *           latch double buffered registers
 *           increment frame counter (ctg+)
 *           generate start of vblank interrupt (gen4+)
 *           |
 *           |          frame start:
 *           |          generate frame start interrupt (aka. vblank interrupt) (gmch)
 *           |          may be shifted forward 1-3 extra lines via PIPECONF
 *           |          |
 *           |          |  start of vsync:
 *           |          |  generate vsync interrupt
 *           |          |  |
 * ___xxxx___    ___xxxx___    ___xxxx___    ___xxxx___    ___xxxx___    ___xxxx
 *       .   \hs/   .      \hs/          \hs/          \hs/   .      \hs/
 * ----va---> <-----------------vb--------------------> <--------va-------------
 *       |          |       <----vs----->                     |
 * -vbs-----> <---vbs+1---> <---vbs+2---> <-----0-----> <-----1-----> <-----2--- (scanline counter gen2)
 * -vbs-2---> <---vbs-1---> <---vbs-----> <---vbs+1---> <---vbs+2---> <-----0--- (scanline counter gen3+)
 * -vbs-2---> <---vbs-2---> <---vbs-1---> <---vbs-----> <---vbs+1---> <---vbs+2- (scanline counter hsw+ hdmi)
 *       |          |                                         |
 *       last visible pixel                                   first visible pixel
 *                  |                                         increment frame counter (gen3/4)
 *                  pixel counter = vblank_start * htotal     pixel counter = 0 (gen3/4)
 *
 * x  = horizontal active
 * _  = horizontal blanking
 * hs = horizontal sync
 * va = vertical active
 * vb = vertical blanking
 * vs = vertical sync
 * vbs = vblank_start (number)
 *
 * Summary:
 * - most events happen at the start of horizontal sync
 * - frame start happens at the start of horizontal blank, 1-4 lines
 *   (depending on PIPECONF settings) after the start of vblank
 * - gen3/4 pixel and frame counter are synchronized with the start
 *   of horizontal active on the first line of vertical active
 */

static u32 i8xx_get_vblank_counter(struct drm_device *dev, int pipe)
{
	/* Gen2 doesn't have a hardware frame counter */
	return 0;
}

/* Called from drm generic code, passed a 'crtc', which
 * we use as a pipe index
 */
static u32 i915_get_vblank_counter(struct drm_device *dev, int pipe)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long high_frame;
	unsigned long low_frame;
	u32 high1, high2, low, pixel, vbl_start, hsync_start, htotal;

	if (!i915_pipe_enabled(dev, pipe)) {
		DRM_DEBUG_DRIVER("trying to get vblank count for disabled "
				"pipe %c\n", pipe_name(pipe));
		return 0;
	}

	if (drm_core_check_feature(dev, DRIVER_MODESET)) {
		struct intel_crtc *intel_crtc =
			to_intel_crtc(dev_priv->pipe_to_crtc_mapping[pipe]);
		const struct drm_display_mode *mode =
			&intel_crtc->config.adjusted_mode;

		htotal = mode->crtc_htotal;
		hsync_start = mode->crtc_hsync_start;
		vbl_start = mode->crtc_vblank_start;
		if (mode->flags & DRM_MODE_FLAG_INTERLACE)
			vbl_start = DIV_ROUND_UP(vbl_start, 2);
	} else {
		enum transcoder cpu_transcoder = (enum transcoder) pipe;

		htotal = ((I915_READ(HTOTAL(cpu_transcoder)) >> 16) & 0x1fff) + 1;
		hsync_start = (I915_READ(HSYNC(cpu_transcoder))  & 0x1fff) + 1;
		vbl_start = (I915_READ(VBLANK(cpu_transcoder)) & 0x1fff) + 1;
		if ((I915_READ(PIPECONF(cpu_transcoder)) &
		     PIPECONF_INTERLACE_MASK) != PIPECONF_PROGRESSIVE)
			vbl_start = DIV_ROUND_UP(vbl_start, 2);
	}

	/* Convert to pixel count */
	vbl_start *= htotal;

	/* Start of vblank event occurs at start of hsync */
	vbl_start -= htotal - hsync_start;

	high_frame = PIPEFRAME(pipe);
	low_frame = PIPEFRAMEPIXEL(pipe);

	/*
	 * High & low register fields aren't synchronized, so make sure
	 * we get a low value that's stable across two reads of the high
	 * register.
	 */
	do {
		high1 = I915_READ(high_frame) & PIPE_FRAME_HIGH_MASK;
		low   = I915_READ(low_frame);
		high2 = I915_READ(high_frame) & PIPE_FRAME_HIGH_MASK;
	} while (high1 != high2);

	high1 >>= PIPE_FRAME_HIGH_SHIFT;
	pixel = low & PIPE_PIXEL_MASK;
	low >>= PIPE_FRAME_LOW_SHIFT;

	/*
	 * The frame counter increments at beginning of active.
	 * Cook up a vblank counter by also checking the pixel
	 * counter against vblank start.
	 */
	return (((high1 << 8) | low) + (pixel >= vbl_start)) & 0xffffff;
}

static u32 gm45_get_vblank_counter(struct drm_device *dev, int pipe)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int reg = PIPE_FRMCOUNT_GM45(pipe);

	if (!i915_pipe_enabled(dev, pipe)) {
		DRM_DEBUG_DRIVER("trying to get vblank count for disabled "
				 "pipe %c\n", pipe_name(pipe));
		return 0;
	}

	return I915_READ(reg);
}

/* raw reads, only for fast reads of display block, no need for forcewake etc. */
#define __raw_i915_read32(dev_priv__, reg__) readl((dev_priv__)->regs + (reg__))

static int __intel_get_crtc_scanline(struct intel_crtc *crtc)
{
	struct drm_device *dev = crtc->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	const struct drm_display_mode *mode;
	enum pipe pipe = crtc->pipe;
	int position, vtotal;

	if (!crtc->active)
		return 0;

	mode = &crtc->config.adjusted_mode;
	vtotal = mode->crtc_vtotal;
	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		vtotal /= 2;

	if (IS_GEN2(dev))
		position = __raw_i915_read32(dev_priv, PIPEDSL(pipe)) & DSL_LINEMASK_GEN2;
	else
		position = __raw_i915_read32(dev_priv, PIPEDSL(pipe)) & DSL_LINEMASK_GEN3;

	/*
	 * See update_scanline_offset() for the details on the
	 * scanline_offset adjustment.
	 */
	return (position + crtc->scanline_offset) % vtotal;
}

static int i915_get_crtc_scanoutpos(struct drm_device *dev, int pipe,
				    unsigned int flags, int *vpos, int *hpos,
				    ktime_t *stime, ktime_t *etime)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_crtc *crtc = dev_priv->pipe_to_crtc_mapping[pipe];
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	const struct drm_display_mode *mode = &intel_crtc->config.adjusted_mode;
	int position;
	int vbl_start, vbl_end, hsync_start, htotal, vtotal;
	bool in_vbl = true;
	int ret = 0;
	unsigned long irqflags;

	if (!intel_crtc->active) {
		DRM_DEBUG_DRIVER("trying to get scanoutpos for disabled "
				 "pipe %c\n", pipe_name(pipe));
		return 0;
	}

	htotal = mode->crtc_htotal;
	hsync_start = mode->crtc_hsync_start;
	vtotal = mode->crtc_vtotal;
	vbl_start = mode->crtc_vblank_start;
	vbl_end = mode->crtc_vblank_end;

	if (mode->flags & DRM_MODE_FLAG_INTERLACE) {
		vbl_start = DIV_ROUND_UP(vbl_start, 2);
		vbl_end /= 2;
		vtotal /= 2;
	}

	ret |= DRM_SCANOUTPOS_VALID | DRM_SCANOUTPOS_ACCURATE;

	/*
	 * Lock uncore.lock, as we will do multiple timing critical raw
	 * register reads, potentially with preemption disabled, so the
	 * following code must not block on uncore.lock.
	 */
	spin_lock_irqsave(&dev_priv->uncore.lock, irqflags);

	/* preempt_disable_rt() should go right here in PREEMPT_RT patchset. */

	/* Get optional system timestamp before query. */
	if (stime)
		*stime = ktime_get();

	if (IS_GEN2(dev) || IS_G4X(dev) || INTEL_INFO(dev)->gen >= 5) {
		/* No obvious pixelcount register. Only query vertical
		 * scanout position from Display scan line register.
		 */
		position = __intel_get_crtc_scanline(intel_crtc);
	} else {
		/* Have access to pixelcount since start of frame.
		 * We can split this into vertical and horizontal
		 * scanout position.
		 */
		position = (__raw_i915_read32(dev_priv, PIPEFRAMEPIXEL(pipe)) & PIPE_PIXEL_MASK) >> PIPE_PIXEL_SHIFT;

		/* convert to pixel counts */
		vbl_start *= htotal;
		vbl_end *= htotal;
		vtotal *= htotal;

		/*
		 * In interlaced modes, the pixel counter counts all pixels,
		 * so one field will have htotal more pixels. In order to avoid
		 * the reported position from jumping backwards when the pixel
		 * counter is beyond the length of the shorter field, just
		 * clamp the position the length of the shorter field. This
		 * matches how the scanline counter based position works since
		 * the scanline counter doesn't count the two half lines.
		 */
		if (position >= vtotal)
			position = vtotal - 1;

		/*
		 * Start of vblank interrupt is triggered at start of hsync,
		 * just prior to the first active line of vblank. However we
		 * consider lines to start at the leading edge of horizontal
		 * active. So, should we get here before we've crossed into
		 * the horizontal active of the first line in vblank, we would
		 * not set the DRM_SCANOUTPOS_INVBL flag. In order to fix that,
		 * always add htotal-hsync_start to the current pixel position.
		 */
		position = (position + htotal - hsync_start) % vtotal;
	}

	/* Get optional system timestamp after query. */
	if (etime)
		*etime = ktime_get();

	/* preempt_enable_rt() should go right here in PREEMPT_RT patchset. */

	spin_unlock_irqrestore(&dev_priv->uncore.lock, irqflags);

	in_vbl = position >= vbl_start && position < vbl_end;

	/*
	 * While in vblank, position will be negative
	 * counting up towards 0 at vbl_end. And outside
	 * vblank, position will be positive counting
	 * up since vbl_end.
	 */
	if (position >= vbl_start)
		position -= vbl_end;
	else
		position += vtotal - vbl_end;

	if (IS_GEN2(dev) || IS_G4X(dev) || INTEL_INFO(dev)->gen >= 5) {
		*vpos = position;
		*hpos = 0;
	} else {
		*vpos = position / htotal;
		*hpos = position - (*vpos * htotal);
	}

	/* In vblank? */
	if (in_vbl)
		ret |= DRM_SCANOUTPOS_INVBL;

	return ret;
}

int intel_get_crtc_scanline(struct intel_crtc *crtc)
{
	struct drm_i915_private *dev_priv = crtc->base.dev->dev_private;
	unsigned long irqflags;
	int position;

	spin_lock_irqsave(&dev_priv->uncore.lock, irqflags);
	position = __intel_get_crtc_scanline(crtc);
	spin_unlock_irqrestore(&dev_priv->uncore.lock, irqflags);

	return position;
}

static int i915_get_vblank_timestamp(struct drm_device *dev, int pipe,
			      int *max_error,
			      struct timeval *vblank_time,
			      unsigned flags)
{
	struct drm_crtc *crtc;

	if (pipe < 0 || pipe >= INTEL_INFO(dev)->num_pipes) {
		DRM_ERROR("Invalid crtc %d\n", pipe);
		return -EINVAL;
	}

	/* Get drm_crtc to timestamp: */
	crtc = intel_get_crtc_for_pipe(dev, pipe);
	if (crtc == NULL) {
		DRM_ERROR("Invalid crtc %d\n", pipe);
		return -EINVAL;
	}

	if (!crtc->enabled) {
		DRM_DEBUG_KMS("crtc %d is disabled\n", pipe);
		return -EBUSY;
	}

	/* Helper routine in DRM core does all the work: */
	return drm_calc_vbltimestamp_from_scanoutpos(dev, pipe, max_error,
						     vblank_time, flags,
						     crtc,
						     &to_intel_crtc(crtc)->config.adjusted_mode);
}

#ifndef CONFIG_MIPI_DLP
static bool intel_hpd_irq_event(struct drm_device *dev,
				struct drm_connector *connector)
{
	enum drm_connector_status old_status;

	WARN_ON(!mutex_is_locked(&dev->mode_config.mutex));
	old_status = connector->status;

	connector->status = connector->funcs->detect(connector, false);
	if (old_status == connector->status)
		return false;

	DRM_DEBUG_KMS("[CONNECTOR:%d:%s] status updated from %s to %s\n",
		      connector->base.id,
		      connector->name,
		      drm_get_connector_status_name(old_status),
		      drm_get_connector_status_name(connector->status));

	return true;
}
#endif

#ifndef CONFIG_MIPI_DLP
static void intel_hdmi_disable_port(struct drm_device *dev,
				struct drm_connector *connector)
{
	u32 temp;
	struct intel_connector *intel_connector = to_intel_connector(connector);
	struct intel_encoder *intel_encoder = intel_connector->encoder;
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(&intel_encoder->base);
	struct drm_i915_private *dev_priv = dev->dev_private;

	temp = I915_READ(intel_hdmi->hdmi_reg);
	temp &= ~(SDVO_ENABLE | SDVO_AUDIO_ENABLE);
	I915_WRITE(intel_hdmi->hdmi_reg, temp);
	POSTING_READ(intel_hdmi->hdmi_reg);
	intel_hdmi->skip_port_check = true;
}
#endif

/*
 * Handle hotplug events outside the interrupt handler proper.
 */

#if defined (CONFIG_MIPI_DLP) || defined (CONFIG_HDMI_DLP)
extern struct drm_device *gdev;
//don't call me during bootup to prevent screen on during boot up
struct intel_dual_display_status dual_display_status;

#ifdef CONFIG_HDMI_DLP
extern int bladeiii_dlp_power_on(struct drm_device *dev);
extern int bladeiii_dlp_close(struct drm_device *dev);
#endif
int simulate_valleyview_irq_handler(bool enable,bool (*cb)(int))
{
	int ret = -1;
	struct drm_device *dev = gdev;
	struct drm_i915_private *dev_priv = (struct drm_i915_private *) dev->dev_private;

	/* moved out of isr */
	if (cb)
		dual_display_status.call_back = cb;

	mutex_lock(&dual_display_status.dual_display_mutex);
    /* to enable dlp,the dlp should be disabled before and main is on*/
	if(enable)
	{
		if((dual_display_status.external_status != PIPE_OFF)
				||(dual_display_status.primary_status!= PIPE_ON))
		{
			mutex_unlock(&dual_display_status.dual_display_mutex);
			DRM_INFO("primary_status = %d,external_status = %d.\n",
					dual_display_status.primary_status,
					dual_display_status.external_status);
			return ret;
		}
	}
	/*to disable dlp, the dlp should be enabled befroe*/
	else
	{
		if(dual_display_status.external_status != PIPE_ON){
			mutex_unlock(&dual_display_status.dual_display_mutex);
			DRM_INFO("external_status = %d",dual_display_status.external_status);
			return ret;
		}
	}
	switch (dual_display_status.external_status) {
		case PIPE_INIT:
		case PIPE_OFF:
			dual_display_status.external_status = PIPE_ON_PROCESSING;
			ret = 0;
			if (dual_display_status.call_back)
				dual_display_status.call_back(PIPE_ON_PROCESSING);
			dual_display_status.dual_display_enabled = true;
			mutex_unlock(&dual_display_status.dual_display_mutex);
#ifdef CONFIG_HDMI_DLP
			DRM_INFO("%s: do open dlp\n", __func__);
			bladeiii_dlp_power_on(dev);
#endif
			break;

		case PIPE_ON:
			ret = 1;
			dual_display_status.external_status = PIPE_OFF_PROCESSING;
			if (dual_display_status.call_back)
				dual_display_status.call_back(PIPE_OFF_PROCESSING);
			dual_display_status.dual_display_enabled = false;
			mutex_unlock(&dual_display_status.dual_display_mutex);
#ifdef CONFIG_HDMI_DLP
			DRM_INFO("%s:do close dlp\n", __func__);
			cancel_delayed_work_sync(&dev_priv->hotplug_delayed_work);
			bladeiii_dlp_close(dev);
			cancel_delayed_work_sync(&(dual_display_status.delay_work));
#endif
			break;

		default:
			DRM_ERROR("external_status status error %d\n", dual_display_status.external_status);
			mutex_unlock(&dual_display_status.dual_display_mutex);
			return ret;
	}

	//queue_work(dev_priv->hpdwq, &dev_priv->hotplug_work);
	return ret;
}

void dual_display_delay_work(struct work_struct *work)
{
    printk("dlp delay work\n");
    if(dual_display_status.external_status != PIPE_ON)
        simulate_valleyview_irq_handler(1,dual_display_status.call_back);
}
#endif

/*
 * This function is the second half of logic to perform fake
 * disconnect-connect. The first half was the connector setting
 * a flag, returning status as disconnected and queing this function.
 *
 * This function uses simulate_disconnect_connect flag to identify the
 * connector that should be detected again. Since this is executed after
 * a delay if the panel is still plugged in it will be reported as
 * connected to user mode
 */
static void intel_hpd_simulate_reconnect_work(struct work_struct *work)
{
	struct drm_i915_private *dev_priv =
		container_of(work, struct drm_i915_private, simulate_work.work);
	struct drm_device *dev = dev_priv->dev;
	struct intel_encoder *intel_encoder;
	struct intel_connector *intel_connector;
	struct drm_connector *connector;
	struct drm_mode_config *mode_config = &dev->mode_config;

	DRM_DEBUG_KMS("\n");
	mutex_lock(&mode_config->mutex);

	list_for_each_entry(connector, &mode_config->connector_list, head) {
		intel_connector = to_intel_connector(connector);
		if (!intel_connector->simulate_disconnect_connect)
			continue;

		intel_connector->simulate_disconnect_connect = false;

		if (!intel_connector->encoder)
			continue;

		intel_encoder = intel_connector->encoder;

		spin_lock_irq(&dev_priv->irq_lock);
		dev_priv->hpd_event_bits |= (1 << intel_encoder->hpd_pin);
		spin_unlock_irq(&dev_priv->irq_lock);
	}

	mutex_unlock(&mode_config->mutex);

	queue_work(dev_priv->hpdwq, &dev_priv->hotplug_work);
}

static void i915_digport_work_func(struct work_struct *work)
{
	struct drm_i915_private *dev_priv =
		container_of(work, struct drm_i915_private, dig_port_work);
	unsigned long irqflags;
	u32 long_port_mask, short_port_mask;
	struct intel_digital_port *intel_dig_port;
	int i, ret;
	u32 old_bits = 0;

	spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
	long_port_mask = dev_priv->long_hpd_port_mask;
	dev_priv->long_hpd_port_mask = 0;
	short_port_mask = dev_priv->short_hpd_port_mask;
	dev_priv->short_hpd_port_mask = 0;
	spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);

	for (i = 0; i < I915_MAX_PORTS; i++) {
		bool valid = false;
		bool long_hpd = false;
		intel_dig_port = dev_priv->hpd_irq_port[i];
		if (!intel_dig_port || !intel_dig_port->hpd_pulse)
			continue;

		if (long_port_mask & (1 << i))  {
			valid = true;
			long_hpd = true;
		} else if (short_port_mask & (1 << i))
			valid = true;

		if (valid) {
			ret = intel_dig_port->hpd_pulse(intel_dig_port,
							long_hpd);
			if (ret == true) {
				/* if we get true fallback to old school hpd */
				old_bits |= (1 << intel_dig_port->base.hpd_pin);
			}
		}
	}

	if (old_bits) {
		spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
		dev_priv->hpd_event_bits |= old_bits;
		spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);
		schedule_work(&dev_priv->hotplug_work);
	}
}

/*
 * Handle hotplug events outside the interrupt handler proper.
 */
#define I915_REENABLE_HOTPLUG_DELAY (2*60*1000)

static void i915_hotplug_work_func(struct work_struct *work)
{
	struct drm_i915_private *dev_priv =
		container_of(work, struct drm_i915_private, hotplug_work);
	struct drm_device *dev = dev_priv->dev;
#ifndef CONFIG_MIPI_DLP
	struct drm_mode_config *mode_config = &dev->mode_config;
	struct intel_connector *intel_connector;
	struct intel_encoder *intel_encoder;
#endif
	struct drm_connector *connector;
#ifndef CONFIG_MIPI_DLP
	unsigned long irqflags;
	bool hpd_disabled = false;
#endif
	bool changed = false;
#ifndef CONFIG_MIPI_DLP
	u32 hpd_event_bits;
	char *envp[] = {"hdcp_hpd", NULL};
#endif
	/* Disable PSR on Hotplug interrupt */
	intel_vlv_edp_psr_disable(dev);

	DRM_INFO("running encoder hotplug functions\n");
#ifndef CONFIG_MIPI_DLP
	mutex_lock(&mode_config->mutex);
	DRM_DEBUG_KMS("running encoder hotplug functions\n");

	spin_lock_irqsave(&dev_priv->irq_lock, irqflags);

	hpd_event_bits = dev_priv->hpd_event_bits;
	dev_priv->hpd_event_bits = 0;
	list_for_each_entry(connector, &mode_config->connector_list, head) {
		intel_connector = to_intel_connector(connector);
		intel_encoder = intel_connector->encoder;
		if (intel_encoder->hpd_pin > HPD_NONE &&
		    dev_priv->hpd_stats[intel_encoder->hpd_pin].hpd_mark == HPD_MARK_DISABLED &&
		    connector->polled == DRM_CONNECTOR_POLL_HPD) {
			DRM_INFO("HPD interrupt storm detected on connector %s: "
				 "switching from hotplug detection to polling\n",
				connector->name);
			dev_priv->hpd_stats[intel_encoder->hpd_pin].hpd_mark = HPD_DISABLED;
			connector->polled = DRM_CONNECTOR_POLL_CONNECT
				| DRM_CONNECTOR_POLL_DISCONNECT;
			hpd_disabled = true;
		}
		if (hpd_event_bits & (1 << intel_encoder->hpd_pin)) {
			DRM_DEBUG_KMS("Connector %s (pin %i) received hotplug event.\n",
				      connector->name, intel_encoder->hpd_pin);
		}
	}
	 /* if there were no outputs to poll, poll was disabled,
	  * therefore make sure it's enabled when disabling HPD on
	  * some connectors */
	if (hpd_disabled) {
		drm_kms_helper_poll_enable(dev);
		mod_timer(&dev_priv->hotplug_reenable_timer,
			  jiffies + msecs_to_jiffies(I915_REENABLE_HOTPLUG_DELAY));
	}

	spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);

	list_for_each_entry(connector, &mode_config->connector_list, head) {
		intel_connector = to_intel_connector(connector);
		intel_encoder = intel_connector->encoder;
		if (hpd_event_bits & (1 << intel_encoder->hpd_pin)) {
			if (intel_encoder->hot_plug)
				intel_encoder->hot_plug(intel_encoder);
			if (intel_hpd_irq_event(dev, connector)) {
				if (connector->status ==
				    connector_status_disconnected &&
				    intel_encoder->type == INTEL_OUTPUT_HDMI) {
					/*
					 * Disable HDMI port immediately
					 * for HDCP
					 */
					intel_hdmi_disable_port(dev, connector);
				}

				changed = true;
				if ((connector->status == connector_status_connected) &&
						dev_priv->hotplug_delayed_work_enabled)
					queue_delayed_work(dev_priv->hpdwq,
						&dev_priv->hotplug_delayed_work, msecs_to_jiffies(300));
			}
		}
	}

	/* Encoder hotplug fn is supposed to use this, now clear it */
	dev_priv->hotplug_status = 0;
	mutex_unlock(&mode_config->mutex);

	if (changed)
		drm_kms_helper_hotplug_event(dev);

	/* HDCPD needs a uevent, every time when there is a hotplug */
	kobject_uevent_env(&dev_priv->dev->primary->kdev->kobj,
		KOBJ_CHANGE, envp);
#else
    mutex_lock(&dev->mode_config.mutex);

    list_for_each_entry(connector, &dev->mode_config.connector_list, head) {
        enum drm_connector_status old_status;

        if (connector->connector_type == DRM_MODE_CONNECTOR_DSI) {
			old_status = connector->status;
            connector->status = connector->funcs->detect(connector, false);
			DRM_ERROR("dlp status:%d", connector->status);
            DRM_INFO("[CONNECTOR:%d:DSI] status updated from %d to %d\n",
                    connector->base.id,
                    old_status, connector->status);

            if (old_status != connector->status)
                changed = true;
        }
    }//list_for_each_entry

    mutex_unlock(&dev->mode_config.mutex);

	if (changed) {
		/* send a uevent + call fbdev */
		printk("%s sending uevent\n" , __func__);
		drm_sysfs_hotplug_event(dev);
	}
#endif
}

static void i915_probe_hotplug_work_func(struct work_struct *work)
{
	struct drm_i915_private *dev_priv =
		container_of(work, struct drm_i915_private, probe_hotplug_work);

	struct drm_device *dev = dev_priv->dev;
	struct drm_mode_config *mode_config = &dev->mode_config;
	struct intel_encoder *intel_encoder;

	mutex_lock(&mode_config->mutex);
	list_for_each_entry(intel_encoder, &mode_config->encoder_list, base.head)
		if (intel_encoder->hot_plug)
			intel_encoder->hot_plug(intel_encoder);

	mutex_unlock(&mode_config->mutex);
	drm_helper_hpd_irq_event(dev);
}

#ifdef CONFIG_HDMI_DLP
static void i915_hotplug_delayed_work_func(struct work_struct *work)
{
	struct drm_i915_private *dev_priv =
		container_of(to_delayed_work(work), struct drm_i915_private, hotplug_delayed_work);
	struct drm_device *dev = dev_priv->dev;
	struct drm_mode_config *mode_config = &dev->mode_config;
	struct intel_connector *intel_connector;
	struct intel_encoder *intel_encoder;
	struct drm_connector *connector;
	enum pipe pipe;

	list_for_each_entry(connector, &mode_config->connector_list, head) {
		intel_connector = to_intel_connector(connector);
		intel_encoder = intel_connector->encoder;

		if (intel_encoder->hot_plug_delayed_work) {
			if ((intel_encoder->get_state) &&
					intel_encoder->get_state(intel_encoder, &pipe)) {
				intel_encoder->hot_plug_delayed_work(intel_encoder);
			} else {
				queue_delayed_work(dev_priv->hpdwq,
					&dev_priv->hotplug_delayed_work, msecs_to_jiffies(100));
			}
		}
	}
}
#endif

static void intel_hpd_irq_uninstall(struct drm_i915_private *dev_priv)
{
	del_timer_sync(&dev_priv->hotplug_reenable_timer);
	cancel_delayed_work_sync(&dev_priv->simulate_work);
}

static void ironlake_rps_change_irq_handler(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 busy_up, busy_down, max_avg, min_avg;
	u8 new_delay;

	spin_lock(&mchdev_lock);

	I915_WRITE16(MEMINTRSTS, I915_READ(MEMINTRSTS));

	new_delay = dev_priv->ips.cur_delay;

	I915_WRITE16(MEMINTRSTS, MEMINT_EVAL_CHG);
	busy_up = I915_READ(RCPREVBSYTUPAVG);
	busy_down = I915_READ(RCPREVBSYTDNAVG);
	max_avg = I915_READ(RCBMAXAVG);
	min_avg = I915_READ(RCBMINAVG);

	/* Handle RCS change request from hw */
	if (busy_up > max_avg) {
		if (dev_priv->ips.cur_delay != dev_priv->ips.max_delay)
			new_delay = dev_priv->ips.cur_delay - 1;
		if (new_delay < dev_priv->ips.max_delay)
			new_delay = dev_priv->ips.max_delay;
	} else if (busy_down < min_avg) {
		if (dev_priv->ips.cur_delay != dev_priv->ips.min_delay)
			new_delay = dev_priv->ips.cur_delay + 1;
		if (new_delay > dev_priv->ips.min_delay)
			new_delay = dev_priv->ips.min_delay;
	}

	if (ironlake_set_drps(dev, new_delay))
		dev_priv->ips.cur_delay = new_delay;

	spin_unlock(&mchdev_lock);

	return;
}

static void notify_ring(struct drm_device *dev,
			struct intel_engine_cs *ring)
{
	if (!intel_ring_initialized(ring))
		return;

	ring->last_irq_seqno = ring->get_seqno(ring, false);
	trace_i915_gem_request_notify(ring);

	i915_scheduler_handle_irq(ring);

	i915_gem_complete_requests_ring(ring, false);

	if (drm_core_check_feature(dev, DRIVER_MODESET))
		intel_notify_mmio_flip(ring);

	wake_up_all(&ring->irq_queue);
}

static u32 vlv_c0_residency(struct drm_i915_private *dev_priv,
				struct  intel_rps_ei_calc *rps_ei)
{
	u32 cz_ts, cz_freq_khz;
	u32 render_count, media_count;
	u32 elapsed_render, elapsed_media, elapsed_time;
	u32 residency = 0;

	cz_ts = vlv_punit_read(dev_priv, PUNIT_REG_CZ_TIMESTAMP);
	cz_freq_khz = DIV_ROUND_CLOSEST(dev_priv->mem_freq * 1000, 4);

	render_count = I915_READ(VLV_RENDER_C0_COUNT_REG);
	media_count = I915_READ(VLV_MEDIA_C0_COUNT_REG);

	if (rps_ei->cz_ts_ei == 0) {
		rps_ei->cz_ts_ei = cz_ts;
		rps_ei->render_ei_c0 = render_count;
		rps_ei->media_ei_c0 = media_count;

		return dev_priv->rps.cur_freq;
	}

	elapsed_time = cz_ts - rps_ei->cz_ts_ei;
	rps_ei->cz_ts_ei = cz_ts;

	elapsed_render = render_count - rps_ei->render_ei_c0;
	rps_ei->render_ei_c0 = render_count;

	elapsed_media = media_count - rps_ei->media_ei_c0;
	rps_ei->media_ei_c0 = media_count;

	/* Convert all the counters into common unit of milli sec */
	elapsed_time /= VLV_CZ_CLOCK_TO_MILLI_SEC;
	elapsed_render /=  cz_freq_khz;
	elapsed_media /= cz_freq_khz;

	/* Calculate overall C0 residency percentage only
	* if elapsed time is non zero
	*/
	if (elapsed_time) {
		residency =
			((max(elapsed_render, elapsed_media) * 100)
				/ elapsed_time);
	}

	return residency;
}

/**
 * vlv_calc_delay_from_C0_counters - Increase/Decrease freq based on GPU
 * busy-ness calculated from C0 counters of render & media power wells
 * @dev_priv: DRM device private
 *
 */
static u32 vlv_calc_delay_from_C0_counters(struct drm_i915_private *dev_priv)
{
	u32 residency_C0_up = 0, residency_C0_down = 0;
	u8 new_delay;

	dev_priv->rps.ei_interrupt_count++;

	WARN_ON(!mutex_is_locked(&dev_priv->rps.hw_lock));

	if (dev_priv->rps_up_ei.cz_ts_ei == 0) {
		vlv_c0_residency(dev_priv, &dev_priv->rps_up_ei);
		vlv_c0_residency(dev_priv, &dev_priv->rps_down_ei);
		return dev_priv->rps.cur_freq;
	}


	/* To down throttle, C0 residency should be less than down threshold
	* for continous EI intervals. So calculate down EI counters
	* once in VLV_INT_COUNT_FOR_DOWN_EI
	*/
	if (dev_priv->rps.ei_interrupt_count == VLV_INT_COUNT_FOR_DOWN_EI) {

		dev_priv->rps.ei_interrupt_count = 0;

		residency_C0_down =  vlv_c0_residency(dev_priv,
						&dev_priv->rps_down_ei);
	} else {
		residency_C0_up =  vlv_c0_residency(dev_priv,
						&dev_priv->rps_up_ei);
	}

	new_delay = dev_priv->rps.cur_freq;

	/* C0 residency is greater than UP threshold. Increase Frequency */
	if (residency_C0_up >= VLV_RP_UP_EI_THRESHOLD) {

		if (dev_priv->rps.cur_freq < dev_priv->rps.max_freq_softlimit)
			new_delay = dev_priv->rps.cur_freq + 1;

		/*
		 * For better performance, jump directly
		 * to RPe if we're below it.
		 */
		if (new_delay < dev_priv->rps.efficient_freq)
			new_delay = dev_priv->rps.efficient_freq;

	} else if (!dev_priv->rps.ei_interrupt_count &&
			(residency_C0_down < VLV_RP_DOWN_EI_THRESHOLD)) {
		/* This means, C0 residency is less than down threshold over
		* a period of VLV_INT_COUNT_FOR_DOWN_EI. So, reduce the freq
		*/
		if (dev_priv->rps.cur_freq > dev_priv->rps.min_freq_softlimit)
			new_delay = dev_priv->rps.cur_freq - 1;
	}

	return new_delay;
}

void gen6_set_pm_mask(struct drm_i915_private *dev_priv,
			     u32 pm_iir, int new_delay)
{
	if (pm_iir & GEN6_PM_RP_UP_THRESHOLD) {
		if (new_delay >= dev_priv->rps.max_freq_softlimit) {
			/* Mask UP THRESHOLD Interrupts */
			I915_WRITE(GEN6_PMINTRMSK,
				   I915_READ(GEN6_PMINTRMSK) |
				   GEN6_PM_RP_UP_THRESHOLD);
			dev_priv->rps.rp_up_masked = true;
		}
		if (dev_priv->rps.rp_down_masked) {
			/* UnMask DOWN THRESHOLD Interrupts */
			I915_WRITE(GEN6_PMINTRMSK,
				   I915_READ(GEN6_PMINTRMSK) &
				   ~GEN6_PM_RP_DOWN_THRESHOLD);
			dev_priv->rps.rp_down_masked = false;
		}
	} else if (pm_iir & GEN6_PM_RP_DOWN_THRESHOLD) {
		if (new_delay <= dev_priv->rps.min_freq_softlimit) {
			/* Mask DOWN THRESHOLD Interrupts */
			I915_WRITE(GEN6_PMINTRMSK,
				   I915_READ(GEN6_PMINTRMSK) |
				   GEN6_PM_RP_DOWN_THRESHOLD);
			dev_priv->rps.rp_down_masked = true;
		}

		if (dev_priv->rps.rp_up_masked) {
			/* UnMask UP THRESHOLD Interrupts */
			I915_WRITE(GEN6_PMINTRMSK,
				   I915_READ(GEN6_PMINTRMSK) &
				   ~GEN6_PM_RP_UP_THRESHOLD);
			dev_priv->rps.rp_up_masked = false;
		}
	}
}

static void gen6_pm_rps_work(struct work_struct *work)
{
	struct drm_i915_private *dev_priv =
		container_of(work, struct drm_i915_private, rps.work);
	u32 pm_iir;
	int new_delay, adj;

	spin_lock_irq(&dev_priv->irq_lock);
	pm_iir = dev_priv->rps.pm_iir;
	dev_priv->rps.pm_iir = 0;

	if (INTEL_INFO(dev_priv->dev)->gen >= 8)
		bdw_enable_pm_irq(dev_priv, dev_priv->pm_rps_events);
	else {
		/* Make sure not to corrupt PMIMR state used by ringbuffer */
		if (dev_priv->rps.use_RC0_residency_for_turbo)
			snb_enable_pm_irq(dev_priv, GEN6_PM_RP_UP_EI_EXPIRED);
		else
			snb_enable_pm_irq(dev_priv, dev_priv->pm_rps_events);
	}
	spin_unlock_irq(&dev_priv->irq_lock);

	/* Make sure we didn't queue anything we're not going to process. */
	WARN_ON(pm_iir & ~(dev_priv->pm_rps_events | GEN6_PM_RP_UP_EI_EXPIRED));

	if ((pm_iir & (dev_priv->pm_rps_events | GEN6_PM_RP_UP_EI_EXPIRED)) == 0)
		return;

	mutex_lock(&dev_priv->rps.hw_lock);

	/* May have just entered manual mode. */
	if (dev_priv->rps.manual_mode) {
		mutex_unlock(&dev_priv->rps.hw_lock);
		return;
	}

	/* Make sure we have current freq updated properly. Doing this
	 * here becuase, on VLV, P-Unit doesnt garauntee that last requested
	 * freq by driver is actually the current running frequency
	 */

	if (IS_CHERRYVIEW(dev_priv->dev))
		chv_update_rps_cur_delay(dev_priv);

	adj = dev_priv->rps.last_adj;
	if (pm_iir & GEN6_PM_RP_UP_THRESHOLD) {
		if (adj > 0)
			adj *= 2;
		else {
			/* CHV needs even encode values */
			adj = IS_CHERRYVIEW(dev_priv->dev) ? 2 : 1;
		}
		new_delay = dev_priv->rps.cur_freq + adj;

		/*
		 * For better performance, jump directly
		 * to RPe if we're below it.
		 */
		if (new_delay < dev_priv->rps.efficient_freq)
			new_delay = dev_priv->rps.efficient_freq;
	} else if (pm_iir & GEN6_PM_RP_DOWN_TIMEOUT) {
		if (dev_priv->rps.cur_freq > dev_priv->rps.efficient_freq)
			new_delay = dev_priv->rps.efficient_freq;
		else
			new_delay = dev_priv->rps.min_freq_softlimit;
		adj = 0;
	} else if (pm_iir & GEN6_PM_RP_UP_EI_EXPIRED) {
		new_delay = vlv_calc_delay_from_C0_counters(dev_priv);
	} else if (pm_iir & GEN6_PM_RP_DOWN_THRESHOLD) {
		if (adj < 0)
			adj *= 2;
		else {
			/* CHV needs even encode values */
			adj = IS_CHERRYVIEW(dev_priv->dev) ? -2 : -1;
		}
		new_delay = dev_priv->rps.cur_freq + adj;
	} else { /* unknown event */
		new_delay = dev_priv->rps.cur_freq;
	}

	/* sysfs frequency interfaces may have snuck in while servicing the
	 * interrupt
	 */
	new_delay = clamp_t(int, new_delay,
			    dev_priv->rps.min_freq_softlimit,
			    dev_priv->rps.max_freq_softlimit);

	dev_priv->rps.last_adj = new_delay - dev_priv->rps.cur_freq;

	if (IS_VALLEYVIEW(dev_priv->dev))
		valleyview_set_rps(dev_priv->dev, new_delay);
	else
		gen6_set_rps(dev_priv->dev, new_delay);

	mutex_unlock(&dev_priv->rps.hw_lock);
}


/**
 * ivybridge_parity_work - Workqueue called when a parity error interrupt
 * occurred.
 * @work: workqueue struct
 *
 * Doesn't actually do anything except notify userspace. As a consequence of
 * this event, userspace should try to remap the bad rows since statistically
 * it is likely the same row is more likely to go bad again.
 */
static void ivybridge_parity_work(struct work_struct *work)
{
	struct drm_i915_private *dev_priv =
		container_of(work, struct drm_i915_private, l3_parity.error_work);
	u32 error_status, row, bank, subbank;
	char *parity_event[6];
	uint32_t misccpctl;
	unsigned long flags;
	uint8_t slice = 0;

	/* We must turn off DOP level clock gating to access the L3 registers.
	 * In order to prevent a get/put style interface, acquire struct mutex
	 * any time we access those registers.
	 */
	mutex_lock(&dev_priv->dev->struct_mutex);

	/* If we've screwed up tracking, just let the interrupt fire again */
	if (WARN_ON(!dev_priv->l3_parity.which_slice))
		goto out;

	misccpctl = I915_READ(GEN7_MISCCPCTL);
	I915_WRITE(GEN7_MISCCPCTL, misccpctl & ~GEN7_DOP_CLOCK_GATE_ENABLE);
	POSTING_READ(GEN7_MISCCPCTL);

	while ((slice = ffs(dev_priv->l3_parity.which_slice)) != 0) {
		u32 reg;

		slice--;
		if (WARN_ON_ONCE(slice >= NUM_L3_SLICES(dev_priv->dev)))
			break;

		dev_priv->l3_parity.which_slice &= ~(1<<slice);

		reg = GEN7_L3CDERRST1 + (slice * 0x200);

		error_status = I915_READ(reg);
		row = GEN7_PARITY_ERROR_ROW(error_status);
		bank = GEN7_PARITY_ERROR_BANK(error_status);
		subbank = GEN7_PARITY_ERROR_SUBBANK(error_status);

		I915_WRITE(reg, GEN7_PARITY_ERROR_VALID | GEN7_L3CDERRST1_ENABLE);
		POSTING_READ(reg);

		parity_event[0] = I915_L3_PARITY_UEVENT "=1";
		parity_event[1] = kasprintf(GFP_KERNEL, "ROW=%d", row);
		parity_event[2] = kasprintf(GFP_KERNEL, "BANK=%d", bank);
		parity_event[3] = kasprintf(GFP_KERNEL, "SUBBANK=%d", subbank);
		parity_event[4] = kasprintf(GFP_KERNEL, "SLICE=%d", slice);
		parity_event[5] = NULL;

		kobject_uevent_env(&dev_priv->dev->primary->kdev->kobj,
				   KOBJ_CHANGE, parity_event);

		DRM_DEBUG("Parity error: Slice = %d, Row = %d, Bank = %d, Sub bank = %d.\n",
			  slice, row, bank, subbank);

		kfree(parity_event[4]);
		kfree(parity_event[3]);
		kfree(parity_event[2]);
		kfree(parity_event[1]);
	}

	I915_WRITE(GEN7_MISCCPCTL, misccpctl);

out:
	WARN_ON(dev_priv->l3_parity.which_slice);
	spin_lock_irqsave(&dev_priv->irq_lock, flags);
	ilk_enable_gt_irq(dev_priv, GT_PARITY_ERROR(dev_priv->dev));
	spin_unlock_irqrestore(&dev_priv->irq_lock, flags);

	mutex_unlock(&dev_priv->dev->struct_mutex);
}

static void ivybridge_parity_error_irq_handler(struct drm_device *dev, u32 iir)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (!HAS_L3_DPF(dev))
		return;

	spin_lock(&dev_priv->irq_lock);
	ilk_disable_gt_irq(dev_priv, GT_PARITY_ERROR(dev));
	spin_unlock(&dev_priv->irq_lock);

	iir &= GT_PARITY_ERROR(dev);
	if (iir & GT_RENDER_L3_PARITY_ERROR_INTERRUPT_S1)
		dev_priv->l3_parity.which_slice |= 1 << 1;

	if (iir & GT_RENDER_L3_PARITY_ERROR_INTERRUPT)
		dev_priv->l3_parity.which_slice |= 1 << 0;

	queue_work(dev_priv->wq, &dev_priv->l3_parity.error_work);
}

static void ilk_gt_irq_handler(struct drm_device *dev,
			       struct drm_i915_private *dev_priv,
			       u32 gt_iir)
{
	if (gt_iir &
	    (GT_RENDER_USER_INTERRUPT | GT_RENDER_PIPECTL_NOTIFY_INTERRUPT))
		notify_ring(dev, &dev_priv->ring[RCS]);
	if (gt_iir & ILK_BSD_USER_INTERRUPT)
		notify_ring(dev, &dev_priv->ring[VCS]);
}

static void snb_gt_irq_handler(struct drm_device *dev,
			       struct drm_i915_private *dev_priv,
			       u32 gt_iir)
{

	if (gt_iir &
	    (GT_RENDER_USER_INTERRUPT | GT_RENDER_PIPECTL_NOTIFY_INTERRUPT))
		notify_ring(dev, &dev_priv->ring[RCS]);
	if (gt_iir & GT_BSD_USER_INTERRUPT)
		notify_ring(dev, &dev_priv->ring[VCS]);
	if (gt_iir & GT_BLT_USER_INTERRUPT)
		notify_ring(dev, &dev_priv->ring[BCS]);

	if (gt_iir & GT_RENDER_CS_MASTER_ERROR_INTERRUPT)
		i915_handle_error(dev, &dev_priv->ring[RCS].hangcheck, 0,
				  "GT RCS error interrupt 0x%08x", gt_iir);

	if (gt_iir & GT_BSD_CS_ERROR_INTERRUPT)
		i915_handle_error(dev, &dev_priv->ring[VCS].hangcheck, 0,
				  "GT VCS error interrupt 0x%08x", gt_iir);

	if (gt_iir & GT_BLT_CS_ERROR_INTERRUPT)
		i915_handle_error(dev, &dev_priv->ring[BCS].hangcheck, 0,
				  "GT BCS error interrupt 0x%08x", gt_iir);

	if (gt_iir & GT_PARITY_ERROR(dev))
		ivybridge_parity_error_irq_handler(dev, gt_iir);

	if (gt_iir & GT_GEN6_RENDER_WATCHDOG_INTERRUPT) {
		struct intel_engine_cs *ring;
		/* Stop the counter to prevent further interrupts */
		ring = &dev_priv->ring[RCS];
		I915_WRITE(RING_CNTR(ring->mmio_base), RCS_WATCHDOG_DISABLE);
		dev_priv->ring[RCS].hangcheck.watchdog_count++;
		i915_handle_error(dev, &dev_priv->ring[RCS].hangcheck, 1,
				  "Render ring timeout counter exceeded");
	}

	if (gt_iir & GT_GEN6_BSD_WATCHDOG_INTERRUPT) {
		struct intel_engine_cs *ring;
		/* Stop the counter to prevent further interrupts */
		ring = &dev_priv->ring[VCS];
		I915_WRITE(RING_CNTR(ring->mmio_base), VCS_WATCHDOG_DISABLE);
		dev_priv->ring[VCS].hangcheck.watchdog_count++;
		i915_handle_error(dev, &dev_priv->ring[VCS].hangcheck, 1,
				  "Video ring timeout counter exceeded");
	}

	if (gt_iir & GT_RENDER_PERFMON_BUFFER_INTERRUPT) {
		atomic_inc(&dev_priv->perfmon.buffer_interrupts);
		wake_up_all(&dev_priv->perfmon.buffer_queue);
	}
}

static void gen8_rps_irq_handler(struct drm_i915_private *dev_priv, u32 pm_iir)
{
	if ((pm_iir & dev_priv->pm_rps_events) == 0)
		return;

	spin_lock(&dev_priv->irq_lock);
	dev_priv->rps.pm_iir |= pm_iir & dev_priv->pm_rps_events;
	bdw_disable_pm_irq(dev_priv, pm_iir & dev_priv->pm_rps_events);
	spin_unlock(&dev_priv->irq_lock);

	queue_work(dev_priv->wq, &dev_priv->rps.work);
}

static irqreturn_t gen8_gt_irq_handler(struct drm_device *dev,
				       struct drm_i915_private *dev_priv,
				       u32 master_ctl)
{
	struct intel_engine_cs *ring;
	u32 rcs, bcs, vcs, oacs;
	uint32_t tmp = 0;
	irqreturn_t ret = IRQ_NONE;

	if (master_ctl & (GEN8_GT_RCS_IRQ | GEN8_GT_BCS_IRQ)) {
		tmp = I915_READ(GEN8_GT_IIR(0));
		if (tmp) {
			I915_WRITE(GEN8_GT_IIR(0), tmp);
			ret = IRQ_HANDLED;

			rcs = tmp >> GEN8_RCS_IRQ_SHIFT;
			ring = &dev_priv->ring[RCS];
			if (rcs & GT_RENDER_USER_INTERRUPT)
				notify_ring(dev, ring);
			if (rcs & GT_CONTEXT_SWITCH_INTERRUPT)
				intel_execlists_handle_ctx_events(ring, true);
			if (rcs & GT_GEN8_RCS_WATCHDOG_INTERRUPT) {
				struct intel_engine_cs *ring;

				/* Stop the counter to prevent further interrupts */
				ring = &dev_priv->ring[RCS];
				I915_WRITE(RING_CNTR(ring->mmio_base), RCS_WATCHDOG_DISABLE);
				dev_priv->ring[RCS].hangcheck.watchdog_count++;
				i915_handle_error(dev, &dev_priv->ring[RCS].hangcheck, 1,
						  "Render ring timeout counter exceeded");
			}

			bcs = tmp >> GEN8_BCS_IRQ_SHIFT;
			ring = &dev_priv->ring[BCS];
			if (bcs & GT_RENDER_USER_INTERRUPT)
				notify_ring(dev, ring);
			if (bcs & GT_CONTEXT_SWITCH_INTERRUPT)
				intel_execlists_handle_ctx_events(ring, true);
		} else
			DRM_ERROR("The master control interrupt lied (GT0)!\n");
	}

	if (master_ctl & (GEN8_GT_VCS1_IRQ | GEN8_GT_VCS2_IRQ)) {
		tmp = I915_READ(GEN8_GT_IIR(1));
		if (tmp) {
			I915_WRITE(GEN8_GT_IIR(1), tmp);
			ret = IRQ_HANDLED;

			vcs = tmp >> GEN8_VCS1_IRQ_SHIFT;
			ring = &dev_priv->ring[VCS];
			if (vcs & GT_RENDER_USER_INTERRUPT)
				notify_ring(dev, ring);
			if (vcs & GT_CONTEXT_SWITCH_INTERRUPT)
				intel_execlists_handle_ctx_events(ring, true);
			if (vcs & GT_GEN8_VCS_WATCHDOG_INTERRUPT) {
				struct intel_engine_cs *ring;

				/* Stop the counter to prevent further interrupts */
				ring = &dev_priv->ring[VCS];
				I915_WRITE(RING_CNTR(ring->mmio_base), VCS_WATCHDOG_DISABLE);
				dev_priv->ring[VCS].hangcheck.watchdog_count++;
				i915_handle_error(dev, &dev_priv->ring[VCS].hangcheck, 1,
						  "Video ring timeout counter exceeded");
			}

			vcs = tmp >> GEN8_VCS2_IRQ_SHIFT;
			ring = &dev_priv->ring[VCS2];
			if (vcs & GT_RENDER_USER_INTERRUPT)
				notify_ring(dev, ring);
			if (vcs & GT_CONTEXT_SWITCH_INTERRUPT)
				intel_execlists_handle_ctx_events(ring, true);
			if (vcs & GT_GEN8_VCS_WATCHDOG_INTERRUPT) {
				struct intel_engine_cs *ring;

				/* Stop the counter to prevent further interrupts */
				ring = &dev_priv->ring[VCS2];
				I915_WRITE(RING_CNTR(ring->mmio_base), VCS2_WATCHDOG_DISABLE);
				dev_priv->ring[VCS2].hangcheck.watchdog_count++;
				i915_handle_error(dev, &dev_priv->ring[VCS2].hangcheck, 1,
						  "Video ring 2 timeout counter exceeded");
			}
		} else
			DRM_ERROR("The master control interrupt lied (GT1)!\n");
	}

	if (master_ctl & GEN8_GT_PM_IRQ) {
		tmp = I915_READ(GEN8_GT_IIR(2));
		if (tmp & dev_priv->pm_rps_events) {
			I915_WRITE(GEN8_GT_IIR(2),
				   tmp & dev_priv->pm_rps_events);
			ret = IRQ_HANDLED;
			gen8_rps_irq_handler(dev_priv, tmp);
		} else
			DRM_ERROR("The master control interrupt lied (PM)!\n");
	}

	if (master_ctl & GEN8_GT_VECS_IRQ) {
		tmp = I915_READ(GEN8_GT_IIR(3));
		if (tmp) {
			I915_WRITE(GEN8_GT_IIR(3), tmp);
			ret = IRQ_HANDLED;

			vcs = tmp >> GEN8_VECS_IRQ_SHIFT;
			ring = &dev_priv->ring[VECS];
			if (vcs & GT_RENDER_USER_INTERRUPT)
				notify_ring(dev, ring);
			if (vcs & GT_CONTEXT_SWITCH_INTERRUPT)
				intel_execlists_handle_ctx_events(ring, true);
		} else
			DRM_ERROR("The master control interrupt lied (GT3)!\n");
	}

	if (master_ctl &  GEN8_GT_OACS_IRQ) {
		tmp = I915_READ(GEN8_GT_IIR(3));
		if (tmp) {
			I915_WRITE(GEN8_GT_IIR(3), tmp);
			ret = IRQ_HANDLED;

			oacs = tmp >> GEN8_OACS_IRQ_SHIFT;
			if (oacs & GT_RENDER_PERFMON_BUFFER_INTERRUPT) {
				atomic_inc(
					&dev_priv->perfmon.buffer_interrupts);
				wake_up_all(&dev_priv->perfmon.buffer_queue);
			}
		} else
			DRM_ERROR("The master control interrupt lied (GT3)!\n");
	}

	return ret;
}

#define HPD_STORM_DETECT_PERIOD 1000
#define HPD_STORM_THRESHOLD 5

static int pch_port_to_hotplug_shift(enum port port)
{
	switch (port) {
	case PORT_A:
	case PORT_E:
	default:
		return -1;
	case PORT_B:
		return 0;
	case PORT_C:
		return 8;
	case PORT_D:
		return 16;
	}
}

static int i915_port_to_hotplug_shift(enum port port)
{
	switch (port) {
	case PORT_A:
	case PORT_E:
	default:
		return -1;
	case PORT_B:
		return 17;
	case PORT_C:
		return 19;
	case PORT_D:
		return 21;
	}
}

static inline enum port get_port_from_pin(enum hpd_pin pin)
{
	switch (pin) {
	case HPD_PORT_B:
		return PORT_B;
	case HPD_PORT_C:
		return PORT_C;
	case HPD_PORT_D:
		return PORT_D;
	default:
		return PORT_A; /* no hpd */
	}
}

static inline void intel_hpd_irq_handler(struct drm_device *dev,
					 u32 hotplug_trigger,
					 u32 dig_hotplug_reg,
					 const u32 *hpd)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int i;
	enum port port;
	bool storm_detected = false;
	bool queue_dig = false, queue_hp = false;
	u32 dig_shift;
	u32 dig_port_mask = 0;

	if (!hotplug_trigger)
		return;

	DRM_DEBUG_DRIVER("hotplug event received, stat 0x%08x, dig 0x%08x\n",
			 hotplug_trigger, dig_hotplug_reg);

	spin_lock(&dev_priv->irq_lock);
	for (i = 1; i < HPD_NUM_PINS; i++) {
		if (!(hpd[i] & hotplug_trigger))
			continue;

		port = get_port_from_pin(i);
		if (port && dev_priv->hpd_irq_port[port]) {
			bool long_hpd;

			if (HAS_PCH_SPLIT(dev)) {
				dig_shift = pch_port_to_hotplug_shift(port);
				long_hpd = (dig_hotplug_reg >> dig_shift) &
						PORTB_HOTPLUG_LONG_DETECT;
			} else {
				dig_shift = i915_port_to_hotplug_shift(port);
				long_hpd = (hotplug_trigger >> dig_shift) &
						PORTB_HOTPLUG_LONG_DETECT;
			}

			DRM_DEBUG_DRIVER("digital hpd port %d %d\n",
						port, long_hpd);
			/* for long HPD pulses we want to have the digital
			 * queue happen, but we still want HPD storm detection
			 * to function.
			 */
			if (long_hpd) {
				dev_priv->long_hpd_port_mask |= (1 << port);
				dig_port_mask |= hpd[i];
			} else {
				/* for short HPD just trigger the digital
				 * queue
				 */
				dev_priv->short_hpd_port_mask |= (1 << port);
				hotplug_trigger &= ~hpd[i];
			}
			queue_dig = true;
		}
	}

	for (i = 1; i < HPD_NUM_PINS; i++) {
		if (hpd[i] & hotplug_trigger &&
		    dev_priv->hpd_stats[i].hpd_mark == HPD_DISABLED) {
			/*
			 * On GMCH platforms the interrupt mask bits only
			 * prevent irq generation, not the setting of the
			 * hotplug bits itself. So only WARN about unexpected
			 * interrupts on saner platforms.
			 */
			WARN_ONCE(INTEL_INFO(dev)->gen >= 5 && !IS_VALLEYVIEW(dev),
				  "Received HPD interrupt (0x%08x) on pin %d (0x%08x) although disabled\n",
				  hotplug_trigger, i, hpd[i]);

			continue;
		}

		if (!(hpd[i] & hotplug_trigger) ||
		    dev_priv->hpd_stats[i].hpd_mark != HPD_ENABLED)
			continue;

		if (!(dig_port_mask & hpd[i])) {
			dev_priv->hpd_event_bits |= (1 << i);
			queue_hp = true;
		}

		if (!time_in_range(jiffies, dev_priv->hpd_stats[i].hpd_last_jiffies,
				   dev_priv->hpd_stats[i].hpd_last_jiffies
				   + msecs_to_jiffies(HPD_STORM_DETECT_PERIOD))) {
			dev_priv->hpd_stats[i].hpd_last_jiffies = jiffies;
			dev_priv->hpd_stats[i].hpd_cnt = 0;
			DRM_DEBUG_KMS("Received HPD interrupt on PIN %d - cnt: 0\n", i);
		} else if (dev_priv->hpd_stats[i].hpd_cnt > HPD_STORM_THRESHOLD) {
			dev_priv->hpd_stats[i].hpd_mark = HPD_MARK_DISABLED;
			dev_priv->hpd_event_bits &= ~(1 << i);
			DRM_DEBUG_KMS("HPD interrupt storm detected on PIN %d\n", i);
			storm_detected = true;
		} else {
			dev_priv->hpd_stats[i].hpd_cnt++;
			DRM_DEBUG_KMS("Received HPD interrupt on PIN %d - cnt: %d\n", i,
				      dev_priv->hpd_stats[i].hpd_cnt);
		}
	}

	if (storm_detected)
		dev_priv->display.hpd_irq_setup(dev);
	spin_unlock(&dev_priv->irq_lock);

	/*
	 * Our hotplug handler can grab modeset locks (by calling down into the
	 * fb helpers). Hence it must not be run on our own dev-priv->wq work
	 * queue for otherwise the flush_work in the pageflip code will
	 * deadlock.
	 */
	if (queue_dig)
		schedule_work(&dev_priv->dig_port_work);
	if (queue_hp)
		queue_work(dev_priv->hpdwq, &dev_priv->hotplug_work);
}

static void gmbus_irq_handler(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	wake_up_all(&dev_priv->gmbus_wait_queue);
}

static void dp_aux_irq_handler(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	wake_up_all(&dev_priv->gmbus_wait_queue);
}

#if defined(CONFIG_DEBUG_FS)
static void display_pipe_crc_irq_handler(struct drm_device *dev, enum pipe pipe,
					 uint32_t crc0, uint32_t crc1,
					 uint32_t crc2, uint32_t crc3,
					 uint32_t crc4)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_pipe_crc *pipe_crc = &dev_priv->pipe_crc[pipe];
	struct intel_pipe_crc_entry *entry;
	int head, tail;

	spin_lock(&pipe_crc->lock);

	if (!pipe_crc->entries) {
		spin_unlock(&pipe_crc->lock);
		DRM_ERROR("spurious interrupt\n");
		return;
	}

	head = pipe_crc->head;
	tail = pipe_crc->tail;

	if (CIRC_SPACE(head, tail, INTEL_PIPE_CRC_ENTRIES_NR) < 1) {
		spin_unlock(&pipe_crc->lock);
		DRM_ERROR("CRC buffer overflowing\n");
		return;
	}

	entry = &pipe_crc->entries[head];

	entry->frame = dev->driver->get_vblank_counter(dev, pipe);
	entry->crc[0] = crc0;
	entry->crc[1] = crc1;
	entry->crc[2] = crc2;
	entry->crc[3] = crc3;
	entry->crc[4] = crc4;

	head = (head + 1) & (INTEL_PIPE_CRC_ENTRIES_NR - 1);
	pipe_crc->head = head;

	spin_unlock(&pipe_crc->lock);

	wake_up_interruptible(&pipe_crc->wq);
}
#else
static inline void
display_pipe_crc_irq_handler(struct drm_device *dev, enum pipe pipe,
			     uint32_t crc0, uint32_t crc1,
			     uint32_t crc2, uint32_t crc3,
			     uint32_t crc4) {}
#endif


static void hsw_pipe_crc_irq_handler(struct drm_device *dev, enum pipe pipe)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	display_pipe_crc_irq_handler(dev, pipe,
				     I915_READ(PIPE_CRC_RES_1_IVB(pipe)),
				     0, 0, 0, 0);
}

static void ivb_pipe_crc_irq_handler(struct drm_device *dev, enum pipe pipe)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	display_pipe_crc_irq_handler(dev, pipe,
				     I915_READ(PIPE_CRC_RES_1_IVB(pipe)),
				     I915_READ(PIPE_CRC_RES_2_IVB(pipe)),
				     I915_READ(PIPE_CRC_RES_3_IVB(pipe)),
				     I915_READ(PIPE_CRC_RES_4_IVB(pipe)),
				     I915_READ(PIPE_CRC_RES_5_IVB(pipe)));
}

static void i9xx_pipe_crc_irq_handler(struct drm_device *dev, enum pipe pipe)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	uint32_t res1, res2;

	if (INTEL_INFO(dev)->gen >= 3)
		res1 = I915_READ(PIPE_CRC_RES_RES1_I915(pipe));
	else
		res1 = 0;

	if (INTEL_INFO(dev)->gen >= 5 || IS_G4X(dev))
		res2 = I915_READ(PIPE_CRC_RES_RES2_G4X(pipe));
	else
		res2 = 0;

	display_pipe_crc_irq_handler(dev, pipe,
				     I915_READ(PIPE_CRC_RES_RED(pipe)),
				     I915_READ(PIPE_CRC_RES_GREEN(pipe)),
				     I915_READ(PIPE_CRC_RES_BLUE(pipe)),
				     res1, res2);
}

void gen8_flip_interrupt(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (!dev_priv->rps.is_bdw_sw_turbo)
		return;

	bdw_software_turbo(dev);
}

/* The RPS events need forcewake, so we add them to a work queue and mask their
 * IMR bits until the work is done. Other interrupts can be processed without
 * the work queue. */
static void gen6_rps_irq_handler(struct drm_i915_private *dev_priv, u32 pm_iir)
{
	if (pm_iir & dev_priv->pm_rps_events) {
		spin_lock(&dev_priv->irq_lock);
		dev_priv->rps.pm_iir |= pm_iir & dev_priv->pm_rps_events;
		snb_disable_pm_irq(dev_priv, pm_iir & dev_priv->pm_rps_events);
		spin_unlock(&dev_priv->irq_lock);

		queue_work(dev_priv->wq, &dev_priv->rps.work);
	}

	if (pm_iir & GEN6_PM_RP_UP_EI_EXPIRED) {
		spin_lock(&dev_priv->irq_lock);
		dev_priv->rps.pm_iir |= pm_iir & GEN6_PM_RP_UP_EI_EXPIRED;
		snb_disable_pm_irq(dev_priv, pm_iir & GEN6_PM_RP_UP_EI_EXPIRED);
		spin_unlock(&dev_priv->irq_lock);
		DRM_DEBUG_DRIVER("\nQueueing RPS Work - RC6 WA Turbo");

		queue_work(dev_priv->wq, &dev_priv->rps.work);
	}

	if (HAS_VEBOX(dev_priv->dev)) {
		if (pm_iir & PM_VEBOX_USER_INTERRUPT)
			notify_ring(dev_priv->dev, &dev_priv->ring[VECS]);

		if (pm_iir & PM_VEBOX_CS_ERROR_INTERRUPT) {
			DRM_ERROR("VEBOX CS error interrupt 0x%08x\n", pm_iir);
			i915_handle_error(dev_priv->dev,
					  &dev_priv->ring[VECS].hangcheck, 0,
					  "VEBOX CS error interrupt 0x%08x",
					  pm_iir);
		}
	}
}

static bool intel_pipe_handle_vblank(struct drm_device *dev, enum pipe pipe)
{
	struct intel_crtc *crtc;

	if (!drm_handle_vblank(dev, pipe))
		return false;

	crtc = to_intel_crtc(intel_get_crtc_for_pipe(dev, pipe));
	wake_up(&crtc->vbl_wait);

	if (intel_dsi_is_enc_on_crtc_cmd_mode(&crtc->base))
		crtc->te_int = true;

	return true;
}

#ifdef CONFIG_SUPPORT_LPDMA_HDMI_AUDIO
static inline
void i915_notify_audio_buffer_status(struct drm_device *dev, u32 reg)
{
	u32 lpe_stream = 0;
	struct drm_i915_private *dev_priv = dev->dev_private;
	lpe_stream = I915_READ(reg);
	if (lpe_stream & I915_HDMI_AUDIO_UNDERRUN) {
		I915_WRITE(reg, I915_HDMI_AUDIO_UNDERRUN);
		mid_hdmi_audio_signal_event(dev,
			HAD_EVENT_AUDIO_BUFFER_UNDERRUN);
	}
	if (lpe_stream & I915_HDMI_AUDIO_BUFFER_DONE) {
		I915_WRITE(reg, I915_HDMI_AUDIO_BUFFER_DONE);
		mid_hdmi_audio_signal_event(dev,
			HAD_EVENT_AUDIO_BUFFER_DONE);
	}
}
#endif

static void valleyview_pipestat_irq_handler(struct drm_device *dev, u32 iir)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_crtc *crtc;
	struct intel_crtc *intel_crtc;
	u32 pipe_stats[I915_MAX_PIPES] = { };
	u32 vlv_psr_status[I915_MAX_PIPES] = {PIPE_A_PSR_STATUS_VLV,
					PIPE_B_PSR_STATUS_VLV, 0};
	int pipe;

	spin_lock(&dev_priv->irq_lock);
	for_each_pipe(pipe) {
		int reg;
		u32 mask, iir_bit = 0;

		/*
		 * PIPESTAT bits get signalled even when the interrupt is
		 * disabled with the mask bits, and some of the status bits do
		 * not generate interrupts at all (like the underrun bit). Hence
		 * we need to be careful that we only handle what we want to
		 * handle.
		 */
		mask = 0;
		if (__cpu_fifo_underrun_reporting_enabled(dev, pipe))
			mask |= PIPE_FIFO_UNDERRUN_STATUS;

		switch (pipe) {
		case PIPE_A:
			iir_bit = I915_DISPLAY_PIPE_A_EVENT_INTERRUPT;
			break;
		case PIPE_B:
			iir_bit = I915_DISPLAY_PIPE_B_EVENT_INTERRUPT;
			break;
		case PIPE_C:
			iir_bit = I915_DISPLAY_PIPE_C_EVENT_INTERRUPT;
			break;
		}
		if (iir & iir_bit)
			mask |= dev_priv->pipestat_irq_mask[pipe];

		if (!mask)
			continue;

		reg = PIPESTAT(pipe);
		mask |= PIPESTAT_INT_ENABLE_MASK;
		pipe_stats[pipe] = I915_READ(reg) & mask;

		/*
		 * Clear the PIPE*STAT regs before the IIR
		 */
		if (pipe_stats[pipe] & (PIPE_FIFO_UNDERRUN_STATUS |
					PIPESTAT_INT_STATUS_MASK))
			I915_WRITE(reg, pipe_stats[pipe]);
	}
	spin_unlock(&dev_priv->irq_lock);

	for_each_pipe(pipe) {
		if (pipe_stats[pipe] & PIPE_START_VBLANK_INTERRUPT_STATUS)
			intel_pipe_handle_vblank(dev, pipe);
		if (pipe_stats[pipe] & PLANE_FLIP_DONE_INT_STATUS_VLV) {
			crtc = intel_get_crtc_for_pipe(dev, pipe);
			intel_crtc = to_intel_crtc(crtc);
			if (intel_crtc->unpin_work) {
				intel_prepare_page_flip(dev, pipe);
				intel_finish_page_flip(dev, pipe);
			}
		}
		if (pipe_stats[pipe] & SPRITE0_FLIP_DONE_INT_STATUS_VLV) {
			intel_prepare_sprite_page_flip(dev, pipe);
			intel_finish_sprite_page_flip(dev, pipe);
		}
		if (pipe_stats[pipe] & SPRITE1_FLIP_DONE_INT_STATUS_VLV) {
			intel_prepare_sprite_page_flip(dev, pipe);
			intel_finish_sprite_page_flip(dev, pipe);
		}
		if (pipe_stats[pipe] & PIPE_CRC_DONE_INTERRUPT_STATUS)
			i9xx_pipe_crc_irq_handler(dev, pipe);

		if (pipe_stats[pipe] & PIPE_DPST_EVENT_STATUS)
			i915_dpst_irq_handler(dev, pipe);

		/* Handle PSR active entry events */
		if (pipe_stats[pipe] & vlv_psr_status[pipe])
			intel_vlv_psr_irq_handler(dev, pipe);

		if (pipe_stats[pipe] & PIPE_FIFO_UNDERRUN_STATUS &&
		    intel_set_cpu_fifo_underrun_reporting(dev, pipe, false))
			DRM_ERROR("pipe %c underrun\n", pipe_name(pipe));
	}

#ifdef CONFIG_SUPPORT_LPDMA_HDMI_AUDIO
	if (IS_CHERRYVIEW(dev)) {
		if (iir & I915_LPE_PIPE_C_INTERRUPT)
			i915_notify_audio_buffer_status(dev,
						I915_LPE_AUDIO_HDMI_STATUS_C);
		if (iir & CHV_I915_LPE_PIPE_B_INTERRUPT)
			i915_notify_audio_buffer_status(dev,
						I915_LPE_AUDIO_HDMI_STATUS_B);
		if (iir & CHV_I915_LPE_PIPE_A_INTERRUPT)
			i915_notify_audio_buffer_status(dev,
						I915_LPE_AUDIO_HDMI_STATUS_A);
	} else {
		if (iir & I915_LPE_PIPE_B_INTERRUPT)
			i915_notify_audio_buffer_status(dev,
						I915_LPE_AUDIO_HDMI_STATUS_B);
	}
#endif

	if (pipe_stats[0] & PIPE_GMBUS_INTERRUPT_STATUS)
		gmbus_irq_handler(dev);
}

static void i9xx_hpd_irq_handler(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 hotplug_status = I915_READ(PORT_HOTPLUG_STAT);

	if (IS_G4X(dev)) {
		u32 hotplug_trigger = hotplug_status & HOTPLUG_INT_STATUS_G4X;

		intel_hpd_irq_handler(dev, hotplug_trigger, 0, hpd_status_g4x);
	} else {
		u32 hotplug_trigger = hotplug_status & HOTPLUG_INT_STATUS_I915;

		/*
		 * Few display's cant set the status for long
		 * time. Lets save this status for future
		 * references like in bottom halves
		 */
#ifdef CONFIG_HDMI_DLP
		if (IS_VALLEYVIEW(dev)) {
			if (hotplug_trigger & HPD_LONG_PULSE) {
				dev_priv->hotplug_status = hotplug_status;
				intel_hpd_irq_handler(dev, hotplug_trigger, 0, hpd_status_i915);
			}
		} else {
			intel_hpd_irq_handler(dev, hotplug_trigger, 0, hpd_status_i915);
		}
#else
		if (IS_VALLEYVIEW(dev)) {
			dev_priv->hotplug_status = hotplug_status;

		intel_hpd_irq_handler(dev, hotplug_trigger, 0, hpd_status_i915);
#endif
	}

	if ((IS_G4X(dev) || IS_VALLEYVIEW(dev)) &&
	    hotplug_status & DP_AUX_CHANNEL_MASK_INT_STATUS_G4X)
		dp_aux_irq_handler(dev);

	I915_WRITE(PORT_HOTPLUG_STAT, hotplug_status);
	/*
	 * Make sure hotplug status is cleared before we clear IIR, or else we
	 * may miss hotplug events.
	 */
	POSTING_READ(PORT_HOTPLUG_STAT);
}

static irqreturn_t valleyview_irq_handler(int irq, void *arg)
{
	struct drm_device *dev = arg;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 iir, gt_iir, pm_iir;
	irqreturn_t ret = IRQ_NONE;
#ifdef CONFIG_MIPI_DLP
     u32 reg;
#endif
	while (true) {
		iir = I915_READ(VLV_IIR);
		gt_iir = I915_READ(GTIIR);
		pm_iir = I915_READ(GEN6_PMIIR);

		if (gt_iir == 0 && pm_iir == 0 && iir == 0)
			goto out;

		ret = IRQ_HANDLED;

		snb_gt_irq_handler(dev, dev_priv, gt_iir);

#ifdef CONFIG_MIPI_DLP
		reg = I915_READ(MIPI_INTR_STAT(0));
		if ((reg &(1<<22))) {
			I915_WRITE(MIPI_INTR_STAT(0), (1<<22));
		}
#endif
		if (i915.enable_intel_adf) {
			if (g_adf_ready && i915.disable_display && iir) {
				unsigned long irqflags;
				spin_lock_irqsave(&dev_priv->irq_lock,
						  irqflags);
				intel_adf_context_on_event();
				spin_unlock_irqrestore(&dev_priv->irq_lock,
						       irqflags);
			} else {
				/*
				 * Clear Pipestat register before clearing iir
				 * As of now only DSI pipe is enabled.
				 * Loop later when we enable HDMI pipe to done
				 * same for both PIPESTAT.
				 */

				I915_WRITE(PIPESTAT(0), I915_READ(PIPESTAT(0)) |
					   I915_DISPLAY_PIPE_A_EVENT_INTERRUPT);
			}
		} else {
			valleyview_pipestat_irq_handler(dev, iir);
			/* Consume port.  Then clear IIR or we'll miss events */
			if (iir & I915_DISPLAY_PORT_INTERRUPT)
				i9xx_hpd_irq_handler(dev);
		}
		if (pm_iir & (GEN6_PM_RPS_EVENTS | GEN6_PM_RP_UP_EI_EXPIRED))
			gen6_rps_irq_handler(dev_priv, pm_iir);

		I915_WRITE(GTIIR, gt_iir);
		I915_WRITE(GEN6_PMIIR, pm_iir);
		I915_WRITE(VLV_IIR, iir);
	}

out:
	return ret;
}

static irqreturn_t cherryview_irq_handler(int irq, void *arg)
{
	struct drm_device *dev = arg;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 master_ctl, count, iir;
	u32 mask, pipestat, pipestat_val;
	irqreturn_t ret = IRQ_NONE;

	for (;;) {
		master_ctl = I915_READ(GEN8_MASTER_IRQ) & ~GEN8_MASTER_IRQ_CONTROL;
		iir = I915_READ(VLV_IIR);

		if (master_ctl == 0 && iir == 0)
			break;

		I915_WRITE(GEN8_MASTER_IRQ, 0);

		gen8_gt_irq_handler(dev, dev_priv, master_ctl);

		if (i915.enable_intel_adf) {
			if (g_adf_ready && i915.disable_display && iir) {
				unsigned long irqflags;
				spin_lock_irqsave(&dev_priv->irq_lock,
						irqflags);
				intel_adf_context_on_event();
				spin_unlock_irqrestore(&dev_priv->irq_lock,
						irqflags);
			} else {
				/*
				 * ADF driver is still initializing, so lets
				 * ignore dispaly interrupts for now
				 */

				count = PIPE_A;
				while (count < I915_MAX_PIPES) {
					mask = PIPESTAT_IIR(count);
					pipestat = PIPESTAT(count);
					pipestat_val = I915_READ(pipestat);
					if (pipestat_val)
						I915_WRITE(pipestat,
							pipestat_val | mask);
					count++;
				}
			}
		} else {
			valleyview_pipestat_irq_handler(dev, iir);

			/* Consume port. Then clear IIR or we'll miss events */
			i9xx_hpd_irq_handler(dev);
		}

		I915_WRITE(VLV_IIR, iir);

		I915_WRITE(GEN8_MASTER_IRQ, DE_MASTER_IRQ_CONTROL);
		POSTING_READ(GEN8_MASTER_IRQ);

		ret = IRQ_HANDLED;
	}

	return ret;
}

static void ibx_irq_handler(struct drm_device *dev, u32 pch_iir)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int pipe;
	u32 hotplug_trigger = pch_iir & SDE_HOTPLUG_MASK;
	u32 dig_hotplug_reg;

	dig_hotplug_reg = I915_READ(PCH_PORT_HOTPLUG);
	I915_WRITE(PCH_PORT_HOTPLUG, dig_hotplug_reg);

	intel_hpd_irq_handler(dev, hotplug_trigger, dig_hotplug_reg, hpd_ibx);

	if (pch_iir & SDE_AUDIO_POWER_MASK) {
		int port = ffs((pch_iir & SDE_AUDIO_POWER_MASK) >>
			       SDE_AUDIO_POWER_SHIFT);
		DRM_DEBUG_DRIVER("PCH audio power change on port %d\n",
				 port_name(port));
	}

	if (pch_iir & SDE_AUX_MASK)
		dp_aux_irq_handler(dev);

	if (pch_iir & SDE_GMBUS)
		gmbus_irq_handler(dev);

	if (pch_iir & SDE_AUDIO_HDCP_MASK)
		DRM_DEBUG_DRIVER("PCH HDCP audio interrupt\n");

	if (pch_iir & SDE_AUDIO_TRANS_MASK)
		DRM_DEBUG_DRIVER("PCH transcoder audio interrupt\n");

	if (pch_iir & SDE_POISON)
		DRM_ERROR("PCH poison interrupt\n");

	if (pch_iir & SDE_FDI_MASK)
		for_each_pipe(pipe)
			DRM_DEBUG_DRIVER("  pipe %c FDI IIR: 0x%08x\n",
					 pipe_name(pipe),
					 I915_READ(FDI_RX_IIR(pipe)));

	if (pch_iir & (SDE_TRANSB_CRC_DONE | SDE_TRANSA_CRC_DONE))
		DRM_DEBUG_DRIVER("PCH transcoder CRC done interrupt\n");

	if (pch_iir & (SDE_TRANSB_CRC_ERR | SDE_TRANSA_CRC_ERR))
		DRM_DEBUG_DRIVER("PCH transcoder CRC error interrupt\n");

	if (pch_iir & SDE_TRANSA_FIFO_UNDER)
		if (intel_set_pch_fifo_underrun_reporting(dev, TRANSCODER_A,
							  false))
			DRM_ERROR("PCH transcoder A FIFO underrun\n");

	if (pch_iir & SDE_TRANSB_FIFO_UNDER)
		if (intel_set_pch_fifo_underrun_reporting(dev, TRANSCODER_B,
							  false))
			DRM_ERROR("PCH transcoder B FIFO underrun\n");
}

static void ivb_err_int_handler(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 err_int = I915_READ(GEN7_ERR_INT);
	enum pipe pipe;

	if (err_int & ERR_INT_POISON)
		DRM_ERROR("Poison interrupt\n");

	for_each_pipe(pipe) {
		if (err_int & ERR_INT_FIFO_UNDERRUN(pipe)) {
			if (intel_set_cpu_fifo_underrun_reporting(dev, pipe,
								  false))
				DRM_ERROR("Pipe %c FIFO underrun\n",
					  pipe_name(pipe));
		}

		if (err_int & ERR_INT_PIPE_CRC_DONE(pipe)) {
			if (IS_IVYBRIDGE(dev))
				ivb_pipe_crc_irq_handler(dev, pipe);
			else
				hsw_pipe_crc_irq_handler(dev, pipe);
		}
	}

	I915_WRITE(GEN7_ERR_INT, err_int);
}

static void cpt_serr_int_handler(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 serr_int = I915_READ(SERR_INT);

	if (serr_int & SERR_INT_POISON)
		DRM_ERROR("PCH poison interrupt\n");

	if (serr_int & SERR_INT_TRANS_A_FIFO_UNDERRUN)
		if (intel_set_pch_fifo_underrun_reporting(dev, TRANSCODER_A,
							  false))
			DRM_ERROR("PCH transcoder A FIFO underrun\n");

	if (serr_int & SERR_INT_TRANS_B_FIFO_UNDERRUN)
		if (intel_set_pch_fifo_underrun_reporting(dev, TRANSCODER_B,
							  false))
			DRM_ERROR("PCH transcoder B FIFO underrun\n");

	if (serr_int & SERR_INT_TRANS_C_FIFO_UNDERRUN)
		if (intel_set_pch_fifo_underrun_reporting(dev, TRANSCODER_C,
							  false))
			DRM_ERROR("PCH transcoder C FIFO underrun\n");

	I915_WRITE(SERR_INT, serr_int);
}

static void cpt_irq_handler(struct drm_device *dev, u32 pch_iir)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int pipe;
	u32 hotplug_trigger = pch_iir & SDE_HOTPLUG_MASK_CPT;
	u32 dig_hotplug_reg;

	dig_hotplug_reg = I915_READ(PCH_PORT_HOTPLUG);
	I915_WRITE(PCH_PORT_HOTPLUG, dig_hotplug_reg);

	intel_hpd_irq_handler(dev, hotplug_trigger, dig_hotplug_reg, hpd_cpt);

	if (pch_iir & SDE_AUDIO_POWER_MASK_CPT) {
		int port = ffs((pch_iir & SDE_AUDIO_POWER_MASK_CPT) >>
			       SDE_AUDIO_POWER_SHIFT_CPT);
		DRM_DEBUG_DRIVER("PCH audio power change on port %c\n",
				 port_name(port));
	}

	if (pch_iir & SDE_AUX_MASK_CPT)
		dp_aux_irq_handler(dev);

	if (pch_iir & SDE_GMBUS_CPT)
		gmbus_irq_handler(dev);

	if (pch_iir & SDE_AUDIO_CP_REQ_CPT)
		DRM_DEBUG_DRIVER("Audio CP request interrupt\n");

	if (pch_iir & SDE_AUDIO_CP_CHG_CPT)
		DRM_DEBUG_DRIVER("Audio CP change interrupt\n");

	if (pch_iir & SDE_FDI_MASK_CPT)
		for_each_pipe(pipe)
			DRM_DEBUG_DRIVER("  pipe %c FDI IIR: 0x%08x\n",
					 pipe_name(pipe),
					 I915_READ(FDI_RX_IIR(pipe)));

	if (pch_iir & SDE_ERROR_CPT)
		cpt_serr_int_handler(dev);
}

static void ilk_display_irq_handler(struct drm_device *dev, u32 de_iir)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	enum pipe pipe;

	if (de_iir & DE_AUX_CHANNEL_A)
		dp_aux_irq_handler(dev);

	if (de_iir & DE_GSE)
		intel_opregion_asle_intr(dev);

	if (de_iir & DE_POISON)
		DRM_ERROR("Poison interrupt\n");

	for_each_pipe(pipe) {
		if (de_iir & DE_PIPE_VBLANK(pipe))
			intel_pipe_handle_vblank(dev, pipe);

		if (de_iir & DE_PIPE_FIFO_UNDERRUN(pipe))
			if (intel_set_cpu_fifo_underrun_reporting(dev, pipe, false))
				DRM_ERROR("Pipe %c FIFO underrun\n",
					  pipe_name(pipe));

		if (de_iir & DE_PIPE_CRC_DONE(pipe))
			i9xx_pipe_crc_irq_handler(dev, pipe);

		/* plane/pipes map 1:1 on ilk+ */
		if (de_iir & DE_PLANE_FLIP_DONE(pipe)) {
			intel_prepare_page_flip(dev, pipe);
			intel_finish_page_flip_plane(dev, pipe);
		}
	}

	/* check event from PCH */
	if (de_iir & DE_PCH_EVENT) {
		u32 pch_iir = I915_READ(SDEIIR);

		if (HAS_PCH_CPT(dev))
			cpt_irq_handler(dev, pch_iir);
		else
			ibx_irq_handler(dev, pch_iir);

		/* should clear PCH hotplug event before clear CPU irq */
		I915_WRITE(SDEIIR, pch_iir);
	}

	if (IS_GEN5(dev) && de_iir & DE_PCU_EVENT)
		ironlake_rps_change_irq_handler(dev);
}

static void ivb_display_irq_handler(struct drm_device *dev, u32 de_iir)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	enum pipe pipe;

	if (de_iir & DE_ERR_INT_IVB)
		ivb_err_int_handler(dev);

	if (de_iir & DE_AUX_CHANNEL_A_IVB)
		dp_aux_irq_handler(dev);

	if (de_iir & DE_GSE_IVB)
		intel_opregion_asle_intr(dev);

	if (de_iir & DE_DPST_HISTOGRAM_IVB)
		i915_dpst_irq_handler(dev, PIPE_A);

	for_each_pipe(pipe) {
		if (de_iir & (DE_PIPE_VBLANK_IVB(pipe)))
			intel_pipe_handle_vblank(dev, pipe);

		/* plane/pipes map 1:1 on ilk+ */
		if (de_iir & DE_PLANE_FLIP_DONE_IVB(pipe)) {
			intel_prepare_page_flip(dev, pipe);
			intel_finish_page_flip_plane(dev, pipe);
		}
	}

	/* check event from PCH */
	if (!HAS_PCH_NOP(dev) && (de_iir & DE_PCH_EVENT_IVB)) {
		u32 pch_iir = I915_READ(SDEIIR);

		cpt_irq_handler(dev, pch_iir);

		/* clear PCH hotplug event before clear CPU irq */
		I915_WRITE(SDEIIR, pch_iir);
	}
}

static irqreturn_t ironlake_irq_handler(int irq, void *arg)
{
	struct drm_device *dev = arg;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 de_iir, gt_iir, de_ier, sde_ier = 0;
	irqreturn_t ret = IRQ_NONE;

	/* We get interrupts on unclaimed registers, so check for this before we
	 * do any I915_{READ,WRITE}. */
	intel_uncore_check_errors(dev);

	/* disable master interrupt before clearing iir  */
	de_ier = I915_READ(DEIER);
	I915_WRITE(DEIER, de_ier & ~DE_MASTER_IRQ_CONTROL);
	POSTING_READ(DEIER);

	/* Disable south interrupts. We'll only write to SDEIIR once, so further
	 * interrupts will will be stored on its back queue, and then we'll be
	 * able to process them after we restore SDEIER (as soon as we restore
	 * it, we'll get an interrupt if SDEIIR still has something to process
	 * due to its back queue). */
	if (!HAS_PCH_NOP(dev)) {
		sde_ier = I915_READ(SDEIER);
		I915_WRITE(SDEIER, 0);
		POSTING_READ(SDEIER);
	}

	gt_iir = I915_READ(GTIIR);
	if (gt_iir) {
		if (INTEL_INFO(dev)->gen >= 6)
			snb_gt_irq_handler(dev, dev_priv, gt_iir);
		else
			ilk_gt_irq_handler(dev, dev_priv, gt_iir);
		I915_WRITE(GTIIR, gt_iir);
		ret = IRQ_HANDLED;
	}

	de_iir = I915_READ(DEIIR);
	if (de_iir) {
		if (INTEL_INFO(dev)->gen >= 7)
			ivb_display_irq_handler(dev, de_iir);
		else
			ilk_display_irq_handler(dev, de_iir);
		I915_WRITE(DEIIR, de_iir);
		ret = IRQ_HANDLED;
	}

	if (INTEL_INFO(dev)->gen >= 6) {
		u32 pm_iir = I915_READ(GEN6_PMIIR);
		if (pm_iir) {
			gen6_rps_irq_handler(dev_priv, pm_iir);
			I915_WRITE(GEN6_PMIIR, pm_iir);
			ret = IRQ_HANDLED;
		}
	}

	I915_WRITE(DEIER, de_ier);
	POSTING_READ(DEIER);
	if (!HAS_PCH_NOP(dev)) {
		I915_WRITE(SDEIER, sde_ier);
		POSTING_READ(SDEIER);
	}

	return ret;
}

static irqreturn_t gen8_irq_handler(int irq, void *arg)
{
	struct drm_device *dev = arg;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 master_ctl;
	irqreturn_t ret = IRQ_NONE;
	uint32_t tmp = 0;
	enum pipe pipe;

	master_ctl = I915_READ(GEN8_MASTER_IRQ);
	master_ctl &= ~GEN8_MASTER_IRQ_CONTROL;
	if (!master_ctl)
		return IRQ_NONE;

	I915_WRITE(GEN8_MASTER_IRQ, 0);
	POSTING_READ(GEN8_MASTER_IRQ);

	/* Find, clear, then process each source of interrupt */

	ret = gen8_gt_irq_handler(dev, dev_priv, master_ctl);

	if (master_ctl & GEN8_DE_MISC_IRQ) {
		tmp = I915_READ(GEN8_DE_MISC_IIR);
		if (tmp) {
			I915_WRITE(GEN8_DE_MISC_IIR, tmp);
			ret = IRQ_HANDLED;
			if (tmp & GEN8_DE_MISC_GSE)
				intel_opregion_asle_intr(dev);
			else
				DRM_ERROR("Unexpected DE Misc interrupt\n");
		}
		else
			DRM_ERROR("The master control interrupt lied (DE MISC)!\n");
	}

	if (master_ctl & GEN8_DE_PORT_IRQ) {
		tmp = I915_READ(GEN8_DE_PORT_IIR);
		if (tmp) {
			I915_WRITE(GEN8_DE_PORT_IIR, tmp);
			ret = IRQ_HANDLED;
			if (tmp & GEN8_AUX_CHANNEL_A)
				dp_aux_irq_handler(dev);
			else
				DRM_ERROR("Unexpected DE Port interrupt\n");
		}
		else
			DRM_ERROR("The master control interrupt lied (DE PORT)!\n");
	}

	for_each_pipe(pipe) {
		uint32_t pipe_iir;

		if (!(master_ctl & GEN8_DE_PIPE_IRQ(pipe)))
			continue;

		pipe_iir = I915_READ(GEN8_DE_PIPE_IIR(pipe));
		if (pipe_iir) {
			ret = IRQ_HANDLED;
			I915_WRITE(GEN8_DE_PIPE_IIR(pipe), pipe_iir);
			if (pipe_iir & GEN8_PIPE_VBLANK)
				intel_pipe_handle_vblank(dev, pipe);

			if (pipe_iir & GEN8_PIPE_DPST_INTERRUPT)
				i915_dpst_irq_handler(dev, pipe);


			if (pipe_iir & GEN8_PIPE_PRIMARY_FLIP_DONE) {
				intel_prepare_page_flip(dev, pipe);
				intel_finish_page_flip_plane(dev, pipe);
			}

			if (pipe_iir & GEN8_PIPE_CDCLK_CRC_DONE)
				hsw_pipe_crc_irq_handler(dev, pipe);

			if (pipe_iir & GEN8_PIPE_FIFO_UNDERRUN) {
				if (intel_set_cpu_fifo_underrun_reporting(dev, pipe,
									  false))
					DRM_ERROR("Pipe %c FIFO underrun\n",
						  pipe_name(pipe));
			}

			if (pipe_iir & GEN8_DE_PIPE_IRQ_FAULT_ERRORS) {
				DRM_ERROR("Fault errors on pipe %c\n: 0x%08x",
					  pipe_name(pipe),
					  pipe_iir & GEN8_DE_PIPE_IRQ_FAULT_ERRORS);
			}
		} else
			DRM_ERROR("The master control interrupt lied (DE PIPE)!\n");
	}

	if (!HAS_PCH_NOP(dev) && master_ctl & GEN8_DE_PCH_IRQ) {
		/*
		 * FIXME(BDW): Assume for now that the new interrupt handling
		 * scheme also closed the SDE interrupt handling race we've seen
		 * on older pch-split platforms. But this needs testing.
		 */
		u32 pch_iir = I915_READ(SDEIIR);
		if (pch_iir) {
			I915_WRITE(SDEIIR, pch_iir);
			ret = IRQ_HANDLED;
			cpt_irq_handler(dev, pch_iir);
		} else
			DRM_ERROR("The master control interrupt lied (SDE)!\n");

	}

	I915_WRITE(GEN8_MASTER_IRQ, GEN8_MASTER_IRQ_CONTROL);
	POSTING_READ(GEN8_MASTER_IRQ);

	return ret;
}

static void i915_error_wake_up(struct drm_i915_private *dev_priv,
			       bool reset_completed)
{
	struct intel_engine_cs *ring;
	int i;

	/*
	 * Notify all waiters for GPU completion events that reset state has
	 * been changed, and that they need to restart their wait after
	 * checking for potential errors (and bail out to drop locks if there is
	 * a gpu reset pending so that i915_error_work_func can acquire them).
	 */

	/* Wake up __wait_seqno, potentially holding dev->struct_mutex. */
	for_each_ring(ring, dev_priv, i)
		wake_up_all(&ring->irq_queue);

	/* Wake up intel_crtc_wait_for_pending_flips, holding crtc->mutex. */
	wake_up_all(&dev_priv->pending_flip_queue);

	/*
	 * Signal tasks blocked in i915_gem_wait_for_error that the pending
	 * reset state is cleared.
	 */
	if (reset_completed)
		wake_up_all(&dev_priv->gpu_error.reset_queue);
}

/**
 * i915_error_work_func - do process context error handling work
 * @work: work struct
 *
 * Fire an error uevent so userspace can see that a hang or error
 * was detected.
 */
static void i915_error_work_func(struct work_struct *work)
{
	struct i915_gpu_error *error = container_of(work, struct i915_gpu_error,
						    work);
	struct drm_i915_private *dev_priv =
		container_of(error, struct drm_i915_private, gpu_error);
	struct drm_device *dev = dev_priv->dev;
	struct intel_engine_cs *ring;
	char *error_event[] = { I915_ERROR_UEVENT "=1", NULL };
	char *reset_event[] = { I915_RESET_UEVENT "=1", NULL };
	char *reset_done_event[] = { I915_ERROR_UEVENT "=0", NULL };
	int ret;
	int i;

	mutex_lock(&dev->struct_mutex);

	kobject_uevent_env(&dev->primary->kdev->kobj, KOBJ_CHANGE, error_event);

	/* Check each ring for a pending reset condition */
	for_each_ring(ring, dev_priv, i) {
		/* Skip individual ring reset requests if full_reset requested*/
		if (i915_reset_in_progress(error))
			break;

		if (atomic_read(&dev_priv->ring[i].hangcheck.flags)
		    & DRM_I915_HANGCHECK_RESET) {
			DRM_DEBUG_TDR("resetting %s\n", ring->name);

			ret = i915_handle_hung_ring(dev, i);

			/*
			 * -EAGAIN means that between detecting a hang (and
			 * also determining that the currently submitted
			 * context is stable and valid) and trying to recover
			 * from the hang the current context changed state.
			 * This means that we are probably not completely hung
			 * after all. Just fail and retry by exiting all the
			 * way back and wait for the next hang detection. If we
			 * have a true hang on our hands then we will detect it
			 * again, otherwise we will continue like nothing
			 * happened.
			 */
			if (ret == -EAGAIN) {
				DRM_ERROR("Reset of %s aborted due to " \
					  "change in context submission " \
					  "state - retrying!", ring->name);
				ret = 0;
			}

			if (ret != 0) {
				DRM_ERROR("ring %d reset failed", i);

				/* Force global reset instead */
				atomic_set_mask(
					I915_RESET_IN_PROGRESS_FLAG,
					&dev_priv->gpu_error.reset_counter);
				break;
			}
		}
	}

	/* Release struct->mutex for the full GPU reset. It will take
	* it itself when it needs it */
	mutex_unlock(&dev->struct_mutex);

	/*
	 * Note that there's only one work item which does gpu resets, so we
	 * need not worry about concurrent gpu resets potentially incrementing
	 * error->reset_counter twice. We only need to take care of another
	 * racing irq/hangcheck declaring the gpu dead for a second time. A
	 * quick check for that is good enough: schedule_work ensures the
	 * correct ordering between hang detection and this work item, and since
	 * the reset in-progress bit is only ever set by code outside of this
	 * work we don't need to worry about any other races.
	 */
	if (i915_reset_in_progress(error) && !i915_terminally_wedged(error)) {
		DRM_DEBUG_DRIVER("resetting chip\n");
		/*
		 * In most cases it's guaranteed that we get here with an RPM
		 * reference held, for example because there is a pending GPU
		 * request that won't finish until the reset is done. This
		 * isn't the case at least when we get here by doing a
		 * simulated reset via debugs, so get an RPM reference.
		 */
		intel_runtime_pm_get(dev_priv);
		/*
		 * All state reset _must_ be completed before we update the
		 * reset counter, for otherwise waiters might miss the reset
		 * pending state and not properly drop locks, resulting in
		 * deadlocks with the reset work.
		 */
		ret = i915_reset(dev);

		kobject_uevent_env(&dev->primary->kdev->kobj, KOBJ_CHANGE,
			reset_event);
		intel_display_handle_reset(dev);

		intel_runtime_pm_put(dev_priv);

		if (ret == 0) {
			for_each_ring(ring, dev_priv, i)
				atomic_set(&dev_priv->ring[i].hangcheck.flags,
					   0);

			/*
			 * After all the gem state is reset, increment the reset
			 * counter and wake up everyone waiting for the reset to
			 * complete.
			 *
			 * Since unlock operations are a one-sided barrier only,
			 * we need to insert a barrier here to order any seqno
			 * updates before the counter increment.
			 * The increment clears the RESET_IN_PROGRESS flag.
			 */
			smp_mb__before_atomic();
			atomic_inc(&dev_priv->gpu_error.reset_counter);
		} else {
			/* Terminal wedge condition */
			WARN(1, "i915_reset failed, declaring GPU as wedged!\n");
			atomic_set_mask(I915_WEDGED, &error->reset_counter);
		}
	}

	/*
	 * Note: The wake_up also serves as a memory barrier so that
	 * waiters see the update value of the reset counter atomic_t.
	 */
	i915_error_wake_up(dev_priv, true);

	kobject_uevent_env(&dev->primary->kdev->kobj,
		KOBJ_CHANGE, reset_done_event);

	DRM_DEBUG_TDR("End recovery work\n");
}

static void i915_report_and_clear_eir(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int pipe, i;
	u32 eir;
	uint32_t instdone[I915_NUM_INSTDONE_REG];
	unsigned long flags;

	spin_lock_irqsave(&dev_priv->gpu_error.lock, flags);

	eir = I915_READ(EIR);
	if (!eir)
		goto i915_report_and_clear_eir_exit;

	pr_err("render error detected, EIR: 0x%08x\n", eir);

	i915_get_extra_instdone(dev, instdone, &dev_priv->ring[RCS]);

	if (IS_G4X(dev)) {
		if (eir & (GM45_ERROR_MEM_PRIV | GM45_ERROR_CP_PRIV)) {
			u32 ipeir = I915_READ(IPEIR_I965);

			pr_err("  IPEIR: 0x%08x\n", I915_READ(IPEIR_I965));
			pr_err("  IPEHR: 0x%08x\n", I915_READ(IPEHR_I965));
			for (i = 0; i < ARRAY_SIZE(instdone); i++)
				pr_err("  INSTDONE_%d: 0x%08x\n", i, instdone[i]);
			pr_err("  INSTPS: 0x%08x\n", I915_READ(INSTPS));
			pr_err("  ACTHD: 0x%08x\n", I915_READ(ACTHD_I965));
			I915_WRITE(IPEIR_I965, ipeir);
			POSTING_READ(IPEIR_I965);
		}
		if (eir & GM45_ERROR_PAGE_TABLE) {
			u32 pgtbl_err = I915_READ(PGTBL_ER);
			pr_err("page table error\n");
			pr_err("  PGTBL_ER: 0x%08x\n", pgtbl_err);
			I915_WRITE(PGTBL_ER, pgtbl_err);
			POSTING_READ(PGTBL_ER);
		}
	}

	if (!IS_GEN2(dev)) {
		if (eir & I915_ERROR_PAGE_TABLE) {
			u32 pgtbl_err = I915_READ(PGTBL_ER);
			pr_err("page table error\n");
			pr_err("  PGTBL_ER: 0x%08x\n", pgtbl_err);
			I915_WRITE(PGTBL_ER, pgtbl_err);
			POSTING_READ(PGTBL_ER);
		}
	}

	if (eir & I915_ERROR_MEMORY_REFRESH) {
		pr_err("memory refresh error:\n");
		for_each_pipe(pipe)
			pr_err("pipe %c stat: 0x%08x\n",
			       pipe_name(pipe), I915_READ(PIPESTAT(pipe)));
		/* pipestat has already been acked */
	}
	if (eir & I915_ERROR_INSTRUCTION) {
		pr_err("instruction error\n");
		pr_err("  INSTPM: 0x%08x\n", I915_READ(INSTPM));
		for (i = 0; i < ARRAY_SIZE(instdone); i++)
			pr_err("  INSTDONE_%d: 0x%08x\n", i, instdone[i]);
		if (INTEL_INFO(dev)->gen < 4) {
			u32 ipeir = I915_READ(IPEIR);

			pr_err("  IPEIR: 0x%08x\n", I915_READ(IPEIR));
			pr_err("  IPEHR: 0x%08x\n", I915_READ(IPEHR));
			pr_err("  ACTHD: 0x%08x\n", I915_READ(ACTHD));
			I915_WRITE(IPEIR, ipeir);
			POSTING_READ(IPEIR);
		} else {
			u32 ipeir = I915_READ(IPEIR_I965);

			pr_err("  IPEIR: 0x%08x\n", I915_READ(IPEIR_I965));
			pr_err("  IPEHR: 0x%08x\n", I915_READ(IPEHR_I965));
			pr_err("  INSTPS: 0x%08x\n", I915_READ(INSTPS));
			pr_err("  ACTHD: 0x%08x\n", I915_READ(ACTHD_I965));
			I915_WRITE(IPEIR_I965, ipeir);
			POSTING_READ(IPEIR_I965);
		}
	}

	I915_WRITE(EIR, eir);
	POSTING_READ(EIR);
	eir = I915_READ(EIR);
	if (eir) {
		/*
		 * some errors might have become stuck,
		 * mask them.
		 */
		DRM_ERROR("EIR stuck: 0x%08x, masking\n", eir);
		I915_WRITE(EMR, I915_READ(EMR) | eir);
		I915_WRITE(IIR, I915_RENDER_COMMAND_PARSER_ERROR_INTERRUPT);
	}
i915_report_and_clear_eir_exit:
	spin_unlock_irqrestore(&dev_priv->gpu_error.lock, flags);
}

/**
 * i915_handle_error - handle an error interrupt
 * @dev: drm device
 *
 * Do some basic checking of regsiter state at error interrupt time and
 * dump it to the syslog.  Also call i915_capture_error_state() to make
 * sure we get a record and make it available in debugfs.  Fire a uevent
 * so userspace knows something bad happened (should trigger collection
 * of a ring dump etc.).
 */
void i915_handle_error(struct drm_device *dev, struct intel_ring_hangcheck *hc,
		       int watchdog, const char *fmt, ...)
{
	int full_reset = 0;
	unsigned long cur_time;
	unsigned long last_reset;
	struct drm_i915_private *dev_priv = dev->dev_private;
	va_list args;
	char error_msg[80];

	if (hc) {
		struct intel_engine_cs *ring = &dev_priv->ring[hc->ringid];

		if (!i915_scheduler_is_ring_flying(ring)) {
			DRM_DEBUG_TDR("False hang detection on %s. Aborting!\n", ring->name);
			return;
		}
	}

	va_start(args, fmt);
	vscnprintf(error_msg, sizeof(error_msg), fmt, args);
	va_end(args);

	i915_capture_error_state(dev, error_msg);
	i915_report_and_clear_eir(dev);

	/*
	 * Currently we only support individual ring reset for GEN7 onwards,
	 * older chips will revert to a full reset.
	 * Error interrupts trigger a full reset (hc == NULL)
	 */
	if ((INTEL_INFO(dev)->gen >= 7) && hc) {
		/*
		 * Flag that the ring has hung. It is this flag that is used
		 * later to update the stats to determine which batch caused
		 * the hang
		 */
		atomic_set_mask(DRM_I915_HANGCHECK_HUNG, &hc->flags);

		/* Now determine what type of reset to use to clear the hang */
		if (!watchdog) {
			cur_time = get_seconds();
			last_reset = hc->last_reset;
			hc->last_reset = cur_time;
		}
		if (!watchdog && ((cur_time - last_reset) <
				  i915.ring_reset_min_alive_period)) {
			/* This ring is hanging too frequently.
			* Opt for full-reset instead */
			DRM_DEBUG_TDR("Ring %d hanging too quickly...\r\n",
				hc->ringid);
			full_reset = 1;
		} else {
			/* Flag that we want to try and reset this ring.
			* This can still be overridden by a global
			* reset */
			atomic_set_mask(DRM_I915_HANGCHECK_RESET, &hc->flags);
			DRM_DEBUG_TDR("Reset Ring %d\n", hc->ringid);
		}
	} else
		full_reset = 1;

	/*
	 * Note: We could mark a ring as 'hung' before we are able
	 * to set the reset request flag so it may be recovered by
	 * an existing reset request. That is OK - it just means that
	 * there may be an extra reset
	 */

	if (!hc || full_reset) {
		/*
		 * Flag that we want a global reset. If there are any
		 * individual ring resets pending then they may
		 * still be processed before the global reset request
		 * is noticed by the error work function.
		 */
		atomic_set_mask(I915_RESET_IN_PROGRESS_FLAG,
			&dev_priv->gpu_error.reset_counter);

		DRM_DEBUG_TDR("Full reset of GPU requested\n");
	}

	/*
	 * Wakeup waiting processes so that the reset work function
	 * i915_error_work_func doesn't deadlock trying to grab various
	 * locks. By bumping the reset counter first, the woken
	 * processes will see a reset in progress and back off,
	 * releasing their locks and then wait for the reset completion.
	 * We must do this for _all_ gpu waiters that might hold locks
	 * that the reset work needs to acquire.
	 *
	 * Note: The wake_up serves as the required memory barrier to
	 * ensure that the waiters see the updated value of the reset
	 * counter atomic_t.
	 */
	i915_error_wake_up(dev_priv, false);

	/*
	 * Our reset work can grab modeset locks (since it needs to reset the
	 * state of outstanding pageflips). Hence it must not be run on our own
	 * dev-priv->wq work queue for otherwise the flush_work in the pageflip
	 * code will deadlock.
	 * If error_work is already in the work queue then it will not be added
	 * again. It hasn't yet executed so it will see the reset flags when
	 * it is scheduled. If it isn't in the queue or it is currently
	 * executing then this call will add it to the queue again so that
	 * even if it misses the reset flags during the current call it is
	 * guaranteed to see them on the next call.
	 */
	DRM_DEBUG_TDR("Queue error work...\n");

	schedule_work(&dev_priv->gpu_error.work);
}

static void __always_unused i915_pageflip_stall_check(struct drm_device *dev, int pipe)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_crtc *crtc = dev_priv->pipe_to_crtc_mapping[pipe];
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	struct drm_i915_gem_object *obj;
	struct intel_unpin_work *work;
	unsigned long flags;
	bool stall_detected;

	/* Ignore early vblank irqs */
	if (intel_crtc == NULL)
		return;

	spin_lock_irqsave(&dev->event_lock, flags);
	work = intel_crtc->unpin_work;

	if (work == NULL ||
	    atomic_read(&work->pending) >= INTEL_FLIP_COMPLETE ||
	    !work->enable_stall_check) {
		/* Either the pending flip IRQ arrived, or we're too early. Don't check */
		spin_unlock_irqrestore(&dev->event_lock, flags);
		return;
	}

	/* Potential stall - if we see that the flip has happened, assume a missed interrupt */
	obj = work->pending_flip_obj;
	if (INTEL_INFO(dev)->gen >= 4) {
		int dspsurf = DSPSURF(intel_crtc->plane);
		stall_detected = I915_HI_DISPBASE(I915_READ(dspsurf)) ==
					i915_gem_obj_ggtt_offset(obj);
	} else {
		int dspaddr = DSPADDR(intel_crtc->plane);
		stall_detected = I915_READ(dspaddr) == (i915_gem_obj_ggtt_offset(obj) +
							crtc->y * crtc->primary->fb->pitches[0] +
							crtc->x * crtc->primary->fb->bits_per_pixel/8);
	}

	spin_unlock_irqrestore(&dev->event_lock, flags);

	if (stall_detected) {
		DRM_DEBUG_DRIVER("Pageflip stall detected\n");
		intel_prepare_page_flip(dev, intel_crtc->plane);
	}
}

/* Called from drm generic code, passed 'crtc' which
 * we use as a pipe index
 */
static int i915_enable_vblank(struct drm_device *dev, int pipe)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long irqflags;

	if (!i915_pipe_enabled(dev, pipe))
		return -EINVAL;

	spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
	if (INTEL_INFO(dev)->gen >= 4)
		i915_enable_pipestat(dev_priv, pipe,
				     PIPE_START_VBLANK_INTERRUPT_STATUS);
	else
		i915_enable_pipestat(dev_priv, pipe,
				     PIPE_VBLANK_INTERRUPT_STATUS);
	spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);

	return 0;
}

static int ironlake_enable_vblank(struct drm_device *dev, int pipe)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long irqflags;
	uint32_t bit = (INTEL_INFO(dev)->gen >= 7) ? DE_PIPE_VBLANK_IVB(pipe) :
						     DE_PIPE_VBLANK(pipe);

	if (!i915_pipe_enabled(dev, pipe))
		return -EINVAL;

	spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
	ironlake_enable_display_irq(dev_priv, bit);
	spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);

	return 0;
}

static int valleyview_enable_vblank(struct drm_device *dev, int pipe)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long irqflags;

	if (!i915_pipe_enabled(dev, pipe))
		return -EINVAL;

	spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
	i915_enable_pipestat(dev_priv, pipe,
			     PIPE_START_VBLANK_INTERRUPT_STATUS);
	spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);

	return 0;
}

static int gen8_enable_vblank(struct drm_device *dev, int pipe)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long irqflags;

	if (!i915_pipe_enabled(dev, pipe))
		return -EINVAL;

	spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
	dev_priv->de_irq_mask[pipe] &= ~GEN8_PIPE_VBLANK;
	I915_WRITE(GEN8_DE_PIPE_IMR(pipe), dev_priv->de_irq_mask[pipe]);
	POSTING_READ(GEN8_DE_PIPE_IMR(pipe));
	spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);
	return 0;
}

/* Called from drm generic code, passed 'crtc' which
 * we use as a pipe index
 */
static void i915_disable_vblank(struct drm_device *dev, int pipe)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long irqflags;

	spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
	i915_disable_pipestat(dev_priv, pipe,
			      PIPE_VBLANK_INTERRUPT_STATUS |
			      PIPE_START_VBLANK_INTERRUPT_STATUS);
	spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);
}

#ifdef CONFIG_SUPPORT_LPDMA_HDMI_AUDIO
int i915_enable_hdmi_audio_int(struct drm_device *dev, int pipe)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long irqflags;
	u32 imr, int_bit;

	spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
	imr = I915_READ(VLV_IMR);
	if (IS_CHERRYVIEW(dev_priv->dev)) {
		int_bit = (pipe ? (CHV_I915_LPE_PIPE_B_INTERRUPT >>
					((pipe - 1) * 9)) :
					CHV_I915_LPE_PIPE_A_INTERRUPT);
		imr &= ~int_bit;
	} else {
		/* Audio is on Stream B */
		imr &= ~I915_LPE_PIPE_B_INTERRUPT;
	}
	I915_WRITE(VLV_IMR, imr);
	i915_enable_lpe_pipestat(dev_priv, pipe);
	spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);

	return 0;
}

int i915_disable_hdmi_audio_int(struct drm_device *dev, int pipe)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long irqflags;
	u32 imr, int_bit;

	spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
	imr = I915_READ(VLV_IMR);
	if (IS_CHERRYVIEW(dev_priv->dev)) {
		int_bit = (pipe ? (CHV_I915_LPE_PIPE_B_INTERRUPT >>
					((pipe - 1) * 9)) :
					CHV_I915_LPE_PIPE_A_INTERRUPT);
		imr |= int_bit;
	}
	else
		/* Audio is on Stream B */
		imr |= I915_LPE_PIPE_B_INTERRUPT;
	I915_WRITE(VLV_IMR, imr);
	i915_disable_lpe_pipestat(dev_priv, pipe);
	spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);

	return 0;
}
#endif

void gen8_enable_oa_interrupt(struct drm_i915_private *dev_priv, uint32_t mask)
{
	u32 imr;
	assert_spin_locked(&dev_priv->irq_lock);
	imr = I915_READ(GEN8_OA_IMR);
	imr &= ~(mask << GEN8_OACS_IRQ_SHIFT);
	I915_WRITE(GEN8_OA_IMR, imr);
}

void gen8_disable_oa_interrupt(struct drm_i915_private *dev_priv, uint32_t mask)
{
	u32 imr;
	assert_spin_locked(&dev_priv->irq_lock);
	imr = I915_READ(GEN8_OA_IMR);
	imr |= (mask << GEN8_OACS_IRQ_SHIFT);
	I915_WRITE(GEN8_OA_IMR, imr);
}

static void ironlake_disable_vblank(struct drm_device *dev, int pipe)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long irqflags;
	uint32_t bit = (INTEL_INFO(dev)->gen >= 7) ? DE_PIPE_VBLANK_IVB(pipe) :
						     DE_PIPE_VBLANK(pipe);

	spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
	ironlake_disable_display_irq(dev_priv, bit);
	spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);
}

static void valleyview_disable_vblank(struct drm_device *dev, int pipe)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long irqflags;

	spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
	i915_disable_pipestat(dev_priv, pipe,
			      PIPE_START_VBLANK_INTERRUPT_STATUS);
	spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);
}

static void gen8_disable_vblank(struct drm_device *dev, int pipe)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long irqflags;

	if (!i915_pipe_enabled(dev, pipe))
		return;

	spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
	dev_priv->de_irq_mask[pipe] |= GEN8_PIPE_VBLANK;
	I915_WRITE(GEN8_DE_PIPE_IMR(pipe), dev_priv->de_irq_mask[pipe]);
	POSTING_READ(GEN8_DE_PIPE_IMR(pipe));
	spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);
}

static struct drm_i915_gem_request *
ring_last_request(struct intel_engine_cs *ring)
{
	return list_entry(ring->request_list.prev,
			  struct drm_i915_gem_request, list);
}

static bool kick_ring(struct intel_engine_cs *ring)
{
	struct drm_device *dev = ring->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 tmp = I915_READ_CTL(ring);

	if (tmp & RING_WAIT) {
		DRM_ERROR("Kicking stuck wait on %s\n",
			ring->name);
		I915_WRITE_CTL(ring, tmp);
		ring->hangcheck.action = HANGCHECK_KICK;
		return true;
	}
	return false;
}

/**
 * This function is called when the TDR algorithm detects that the
 * hardware has not advanced during the last sampling period
 */
static bool i915_hangcheck_hung(struct intel_ring_hangcheck *hc, u32 seqno)
{
	struct drm_device *dev = hc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	uint32_t mbox_wait;
	uint32_t threshold;
	struct intel_engine_cs *ring;
	bool force_full_GPU_reset = false;

	DRM_DEBUG_TDR("Ring [%d] hc->count = %d\n", hc->ringid, hc->count);
	ring = &dev_priv->ring[hc->ringid];

	/*
	 * Is this ring waiting on a semaphore mbox?
	 * If so, give it a bit longer as it may be waiting on another
	 * ring which has actually hung. Give the other ring chance to
	 * reset and clear the hang.
	 */
	mbox_wait = I915_READ_CTL(ring);

	threshold = (mbox_wait & RING_WAIT_SEMAPHORE) ?
		DRM_I915_MBOX_HANGCHECK_THRESHOLD :
		DRM_I915_HANGCHECK_THRESHOLD;

	DRM_DEBUG_TDR("mbox_wait = %u threshold = %u", mbox_wait, threshold);

	if (i915.enable_execlists) {
		/*
		 * TODO:
		 * We could trace the context submission status history
		 * from the previous hang checks but the question is
		 * how we would interpret the information. If we're not
		 * stuck in an inconsistent state in every sample
		 * period and it's fluctuating then how do we react? We
		 * might as well try an inconsistency check just before
		 * making a decision on hang recovery and if that gives
		 * an inconsistent result we can't go for engine reset
		 * anyway so there's not much in terms of options at
		 * this point.
		 *
		 * WARNING:
		 * When checking consistency in the hang checker as a
		 * pre-stage to scheduling hang recovery then watchdog
		 * timeout will break if there is an inconsistent state
		 * and the hang checker has been disabled. The way
		 * around this is by moving consistency checking to the
		 * start of the engine hang recovery path which is
		 * shared for all forms of hang recovery.
		 */

		/*
		 * Consistency checking needs to happen before the 3-second
		 * userland timeout happens, which means that we cannot wait
		 * for 3 full hang check sample periods. Instead, do the
		 * consistency check within 2 sample periods, that is if seqno
		 * has not progressed in between two sample periods. Once
		 * i915_hangcheck_hung() is called lack of execution progress
		 * between 2 sample periods has already been detected. In some
		 * cases (e.g. when HEAD is zero due to hardware being idle and
		 * the last recorded HEAD value was zero too we might end up in
		 * i915_hangcheck_hung() as soon as the first hang check sample
		 * period. In order to avoid consistency checking happening
		 * before a preliminary hang detection has been done make sure
		 * to check seqno progress at this point. HEAD might go to zero
		 * and cause false positive hang detections that way but seqnos
		 * will always progress.
		 */
		if (seqno == hc->last_seqno) {

			enum context_submission_status status =
				i915_gem_context_get_current_context(ring, NULL);

			if (status == CONTEXT_SUBMISSION_STATUS_SUBMITTED) {
				DRM_DEBUG_TDR("Inconsistent context state. Faking interrupt on %s!\n", ring->name);
				if (!intel_execlists_TDR_force_CSB_check(dev_priv, hc->ringid)) {
					if (i915.enable_inconsistency_reset) {
						WARN(1, "Inconsistency rectification failed! Falling back to full GPU reset!\n");
						force_full_GPU_reset = true;
					} else {
						WARN(1, "Inconsistency rectification failed! Will not fall back to full GPU reset!\n");
						/*
						 * WARNING:
						 * Without a way to fall back
						 * to full GPU reset in the
						 * case of a theoretical
						 * irrecoverably inconsistent
						 * state it is conceivable that
						 * the device might hang until
						 * reboot.
						 */
						force_full_GPU_reset = false;
					}

				} else {
					/*
					 * Hang was apparently due to inconsistent HW/driver state.
					 * No need for hang recovery right now.
					 */
					hc->count = 0;
					return false;
				}
			}
		}
	}

	if ((hc->count++ > threshold) || force_full_GPU_reset) {
		bool hung = true;

		DRM_DEBUG_TDR("Hang check period expired... %s hung\n",
				ring->name);

		/* Reset the counter */
		hc->count = 0;

		if (!force_full_GPU_reset)
			i915_sync_hung_ring(ring);

		if (!force_full_GPU_reset && !IS_GEN2(dev)) {
			/* If the ring is hanging on a WAIT_FOR_EVENT
			* then simply poke the RB_WAIT bit
			* and break the hang. This should work on
			* all but the second generation chipsets.
			*/
			ring = &dev_priv->ring[hc->ringid];
			hung &= !kick_ring(ring);
			DRM_DEBUG_TDR("hung=%d after kick ring\n", hung);
		}

		if (hung) {
			hc->tdr_count++;
			ring->hangcheck.action = HANGCHECK_HUNG;
			if (force_full_GPU_reset)
				i915_handle_error(dev, NULL, 0, "Full GPU reset fall-back from %s", ring->name);
			else
				i915_handle_error(dev, hc, 0, "%s hung", ring->name);
		}
		return hung;
	}

	return false;
}

/*
 * This is called from the hangcheck work queue for each ring.
 * It samples the current state of the hardware to make
 * sure that it is progressing.
 */
void i915_hangcheck_sample(struct work_struct *work)
{
	bool idle;
	int empty;
	int instdone_cmp;
	int pending_work = 1;
	int resched_timer = 1;
	struct drm_i915_gem_request *last_req = NULL;
	uint32_t head, tail, acthd, instdone[I915_NUM_INSTDONE_REG], seqno;
	struct drm_device *dev;
	struct drm_i915_private *dev_priv;
	struct intel_engine_cs *ring;
	enum context_submission_status status = CONTEXT_SUBMISSION_STATUS_OK;
	struct intel_ring_hangcheck *hc =
		container_of(work, typeof(*hc), work.work);

	if (!i915.enable_hangcheck || !hc)
		return;

	dev = hc->dev;
	dev_priv = dev->dev_private;
	ring = &dev_priv->ring[hc->ringid];

	/* Sample the current state */

	if (i915.enable_execlists)
		status = i915_gem_context_get_current_context(ring, NULL);

	head = I915_READ_HEAD(ring) & HEAD_ADDR;
	tail = I915_READ_TAIL(ring) & TAIL_ADDR;
	acthd = intel_ring_get_active_head(ring);
	empty = list_empty(&ring->request_list);
	seqno = ring->get_seqno(ring, false);

	i915_get_extra_instdone(dev, instdone, ring);
	instdone_cmp = (memcmp(hc->prev_instdone,
		instdone, sizeof(instdone)) == 0) ? 1 : 0;

	if (!empty) {
		/* Examine the request list to see where the HW has got to
		* (Only call ring_last_request when the list is non-empty)*/
		last_req = ring_last_request(ring);
	}

	if (empty || i915_gem_request_completed(last_req)) {
		/* If the request list is empty or the HW has passed the
		* last seqno of the last item in the request list then the
		* HW is considered idle.
		* The driver may not have cleaned up the request list yet */
		pending_work = 0;
	}

	idle = ((head == tail) && (pending_work == 0));

	DRM_DEBUG_TDR("[%u] HD: 0x%08x 0x%08x, ACTHD: 0x%08x 0x%08x IC: %d " \
		      "status: %u\n", ring->id, (unsigned int) head,
		      (unsigned int) hc->last_hd, (unsigned int) acthd,
		      (unsigned int) hc->last_acthd, instdone_cmp, status);
	DRM_DEBUG_TDR("[%u] E:%d PW:%d TL:0x%08x Csq:0x%08x (%ld) Lsq:0x%08x (%ld) Psq:0x%08x (%ld) Idle: %s\n",
		      ring->id, empty, pending_work, (unsigned int) tail,
		      (unsigned int) seqno,
		      (long int) seqno,
		      (unsigned int) i915_gem_request_get_seqno(last_req),
		      (long int) i915_gem_request_get_seqno(last_req),
		      (unsigned int) hc->last_seqno,
		      (long int) hc->last_seqno,
		      (idle ? "true" : "false"));

	/*
	 * ACTHD breaks in some instances by constantly changing even when
	 * there is no reason for doing so. The underlying reason for this is
	 * still pending investigation. This behaviour breaks the TDR hang
	 * detection since it interprets the constantly changing ACTHD as a
	 * liveliness indication even when there is a hang. To work around this
	 * problem we disable the ACTHD check until we have investigated why
	 * this is happening. The hang detection algorithm does not rely on
	 * ACTHD entirely and will work just as well in the vast majority of
	 * cases even without checking ACTHD.
	 */
	acthd = 0;

	/*
	 * INSTDONE has been shown to be very random in its behaviour and does
	 * not give us much information about the state of the command
	 * streamer. It has been observed that the INSTDONE register keeps
	 * changing state even when all command streamers are hung and are in a
	 * stable state. For now disable the INSTDONE register check to make
	 * the hang check algorithm more predictable in its behaviour and hang
	 * check decision time.
	 *
	 * One destructive side-effect of having a somewhat unpredictable hang
	 * detection time is that the Android userland sometimes tends to time
	 * out during hangs, resulting in the display in a frozen state. That
	 * is something we'd like to avoid if possible.
	 */
	instdone_cmp = 1;

	/* Check both head and active head.
	* Neither is enough on its own - acthd can be pointing within the
	* batch buffer so is more likely to be moving, but the same
	* underlying buffer object could be submitted more than once.
	* If it happens to pause at exactly the same place in the batch
	* buffer and we sample it at that moment then we could see it as
	* hung over 3 sample periods that do not belong to the same
	* batch submission - this would result in a false positive.
	* We know that the head pointer will have advanced for each
	* batch buffer as the ring has to contain a new MI_BATCH_BUFFER_START
	* for every do_exec call, so by combining head and active head we can
	* ensure that the hang detection distinguishes between batch buffers*/
	if ((hc->last_acthd == acthd)
	    && (hc->last_hd == head)
	    && instdone_cmp) {
		/* Ring hasn't advanced in this sampling period */
		if (idle && (status != CONTEXT_SUBMISSION_STATUS_NONE_SUBMITTED)) {
			ring->hangcheck.action = HANGCHECK_IDLE;

			/* The hardware is idle */
			if (waitqueue_active(&ring->irq_queue)) {
				/* We expect the wait queue to drain
				* if the hardware has remained idle
				* for 3 consecutive samples. Wake up
				* the queue on each sample to try and
				* release it, but if it persists then
				* trigger a reset */
				DRM_DEBUG_TDR("Possible stuck wait (0x%08x)\n",
					ring->last_irq_seqno);
				wake_up_all(&ring->irq_queue);
				i915_hangcheck_hung(hc, seqno);
			} else {
				/* Hardware and driver both idle */
				hc->count = 0;
				resched_timer = 0;
			}
		} else if (status != CONTEXT_SUBMISSION_STATUS_NONE_SUBMITTED) {
			/*
			 * The hardware is busy but has not advanced
			 * since the last sample - possible hang
			 */
			i915_hangcheck_hung(hc, seqno);
		}
	} else {
		/* The state has changed so the hardware is active */
		hc->count = 0;
		ring->hangcheck.action = HANGCHECK_ACTIVE;
	}

	/* Always update last sampled state */
	hc->last_hd = head;
	hc->last_acthd = acthd;
	hc->last_seqno = seqno;
	memcpy(hc->prev_instdone, instdone, sizeof(instdone));

	resched_timer &= (status != CONTEXT_SUBMISSION_STATUS_NONE_SUBMITTED);

	if (resched_timer) {
		/*
		 * Work is still pending! Reschedule hang check to come back
		 * later and do another round of hang checking.
		 */
		mod_delayed_work(dev_priv->ring[hc->ringid].hangcheck.wq,
				&dev_priv->ring[hc->ringid].hangcheck.work,
				DRM_I915_HANGCHECK_JIFFIES);
	}
}

void i915_queue_hangcheck(struct drm_device *dev, u32 ringid,
		unsigned long retire_work_timestamp)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_engine_cs *ring = &dev_priv->ring[ringid];
	long timediff;
	unsigned long time = 0;

	if (!i915.enable_hangcheck)
		return;

	/*
	 * We are piggy-backing the hang check on top of the retire
	 * work timer so the amount of time we need to wait between
	 * expired retire work timer and the actual hangcheck is:
	 *
	 *	 wait_time = hangcheck_period - retire_work_period
	 *
	 * retire_work_period is the time difference between now
	 * and when the retire_work_timer was scheduled.
	 * If we have already waited the hangcheck period or more
	 * then simply schedule the hang check immediately.
	 *
	 * If a hang check was already rescheduled by a previous
	 * hang check (because there are still pending work that
	 * needs to be checked) and is now pending, this schedule
	 * call will not affect anything and the pending hang check
	 * will be carried out without being postponed.
	 */
	timediff = jiffies - retire_work_timestamp;

	if (DRM_I915_HANGCHECK_JIFFIES > timediff)
		time = DRM_I915_HANGCHECK_JIFFIES - timediff;

	queue_delayed_work(ring->hangcheck.wq, &ring->hangcheck.work, time);
}

static void ibx_irq_reset(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (HAS_PCH_NOP(dev))
		return;

	GEN5_IRQ_RESET(SDE);

	if (HAS_PCH_CPT(dev) || HAS_PCH_LPT(dev))
		I915_WRITE(SERR_INT, 0xffffffff);
}

/*
 * SDEIER is also touched by the interrupt handler to work around missed PCH
 * interrupts. Hence we can't update it after the interrupt handler is enabled -
 * instead we unconditionally enable all PCH interrupt sources here, but then
 * only unmask them as needed with SDEIMR.
 *
 * This function needs to be called before interrupts are enabled.
 */
static void ibx_irq_pre_postinstall(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (HAS_PCH_NOP(dev))
		return;

	WARN_ON(I915_READ(SDEIER) != 0);
	I915_WRITE(SDEIER, 0xffffffff);
	POSTING_READ(SDEIER);
}

static void gen5_gt_irq_reset(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	GEN5_IRQ_RESET(GT);
	if (INTEL_INFO(dev)->gen >= 6)
		GEN5_IRQ_RESET(GEN6_PM);
}

/* drm_dma.h hooks
*/
static void ironlake_irq_reset(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	I915_WRITE(HWSTAM, 0xffffffff);

	GEN5_IRQ_RESET(DE);
	if (IS_GEN7(dev))
		I915_WRITE(GEN7_ERR_INT, 0xffffffff);

	gen5_gt_irq_reset(dev);

	ibx_irq_reset(dev);
}

static void valleyview_irq_preinstall(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int pipe;

	/* VLV magic */
	I915_WRITE(VLV_IMR, 0);
	I915_WRITE(RING_IMR(RENDER_RING_BASE), 0);
	I915_WRITE(RING_IMR(GEN6_BSD_RING_BASE), 0);
	I915_WRITE(RING_IMR(BLT_RING_BASE), 0);

	/* and GT */
	I915_WRITE(GTIIR, I915_READ(GTIIR));
	I915_WRITE(GTIIR, I915_READ(GTIIR));

	gen5_gt_irq_reset(dev);

	I915_WRITE(DPINVGTT, 0xff);

	I915_WRITE(PORT_HOTPLUG_EN, 0);
	I915_WRITE(PORT_HOTPLUG_STAT, I915_READ(PORT_HOTPLUG_STAT));
	for_each_pipe(pipe)
		I915_WRITE(PIPESTAT(pipe), 0xffff);
	I915_WRITE(VLV_IIR, 0xffffffff);
	I915_WRITE(VLV_IMR, 0xffffffff);
	I915_WRITE(VLV_IER, 0x0);
	POSTING_READ(VLV_IER);
}

static void gen8_gt_irq_reset(struct drm_i915_private *dev_priv)
{
	GEN8_IRQ_RESET_NDX(GT, 0);
	GEN8_IRQ_RESET_NDX(GT, 1);
	GEN8_IRQ_RESET_NDX(GT, 2);
	GEN8_IRQ_RESET_NDX(GT, 3);
}

static void gen8_irq_reset(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int pipe;

	I915_WRITE(GEN8_MASTER_IRQ, 0);
	POSTING_READ(GEN8_MASTER_IRQ);

	gen8_gt_irq_reset(dev_priv);

	for_each_pipe(pipe)
		GEN8_IRQ_RESET_NDX(DE_PIPE, pipe);

	GEN5_IRQ_RESET(GEN8_DE_PORT_);
	GEN5_IRQ_RESET(GEN8_DE_MISC_);
	GEN5_IRQ_RESET(GEN8_PCU_);

	ibx_irq_reset(dev);
}

static void cherryview_irq_preinstall(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int pipe;

	I915_WRITE(GEN8_MASTER_IRQ, 0);
	POSTING_READ(GEN8_MASTER_IRQ);

	gen8_gt_irq_reset(dev_priv);

	GEN5_IRQ_RESET(GEN8_PCU_);

	POSTING_READ(GEN8_PCU_IIR);

	I915_WRITE(DPINVGTT, DPINVGTT_STATUS_MASK_CHV);

	I915_WRITE(PORT_HOTPLUG_EN, 0);
	I915_WRITE(PORT_HOTPLUG_STAT, I915_READ(PORT_HOTPLUG_STAT));

	for_each_pipe(pipe)
		I915_WRITE(PIPESTAT(pipe), 0xffff);

	I915_WRITE(VLV_IMR, 0xffffffff);
	I915_WRITE(VLV_IER, 0x0);
	I915_WRITE(VLV_IIR, 0xffffffff);
	POSTING_READ(VLV_IIR);
}

static void ibx_hpd_irq_setup(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_mode_config *mode_config = &dev->mode_config;
	struct intel_encoder *intel_encoder;
	u32 hotplug_irqs, hotplug, enabled_irqs = 0;

	if (HAS_PCH_IBX(dev)) {
		hotplug_irqs = SDE_HOTPLUG_MASK;
		list_for_each_entry(intel_encoder, &mode_config->encoder_list, base.head)
			if (dev_priv->hpd_stats[intel_encoder->hpd_pin].hpd_mark == HPD_ENABLED)
				enabled_irqs |= hpd_ibx[intel_encoder->hpd_pin];
	} else {
		hotplug_irqs = SDE_HOTPLUG_MASK_CPT;
		list_for_each_entry(intel_encoder, &mode_config->encoder_list, base.head)
			if (dev_priv->hpd_stats[intel_encoder->hpd_pin].hpd_mark == HPD_ENABLED)
				enabled_irqs |= hpd_cpt[intel_encoder->hpd_pin];
	}

	ibx_display_interrupt_update(dev_priv, hotplug_irqs, enabled_irqs);

	/*
	 * Enable digital hotplug on the PCH, and configure the DP short pulse
	 * duration to 2ms (which is the minimum in the Display Port spec)
	 *
	 * This register is the same on all known PCH chips.
	 */
	hotplug = I915_READ(PCH_PORT_HOTPLUG);
	hotplug &= ~(PORTD_PULSE_DURATION_MASK|PORTC_PULSE_DURATION_MASK|PORTB_PULSE_DURATION_MASK);
	hotplug |= PORTD_HOTPLUG_ENABLE | PORTD_PULSE_DURATION_2ms;
	hotplug |= PORTC_HOTPLUG_ENABLE | PORTC_PULSE_DURATION_2ms;
	hotplug |= PORTB_HOTPLUG_ENABLE | PORTB_PULSE_DURATION_2ms;
	I915_WRITE(PCH_PORT_HOTPLUG, hotplug);
}

static void ibx_irq_postinstall(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 mask;

	if (HAS_PCH_NOP(dev))
		return;

	if (HAS_PCH_IBX(dev))
		mask = SDE_GMBUS | SDE_AUX_MASK | SDE_POISON;
	else
		mask = SDE_GMBUS_CPT | SDE_AUX_MASK_CPT;

	GEN5_ASSERT_IIR_IS_ZERO(SDEIIR);
	I915_WRITE(SDEIMR, ~mask);
}

static void gen5_gt_irq_postinstall(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 pm_irqs, gt_irqs;

	pm_irqs = gt_irqs = 0;

	dev_priv->gt_irq_mask = ~0;
	if (HAS_L3_DPF(dev)) {
		/* L3 parity interrupt is always unmasked. */
		dev_priv->gt_irq_mask = ~GT_PARITY_ERROR(dev);
		gt_irqs |= GT_PARITY_ERROR(dev);
	}

	if (IS_GEN7(dev))
		gt_irqs |= GT_RENDER_PERFMON_BUFFER_INTERRUPT;

	if (IS_VALLEYVIEW(dev))
		dev_priv->gt_irq_mask &= ~(I915_SYNC_USER_INTERRUPTS);

	gt_irqs |= GT_RENDER_USER_INTERRUPT;

	if (INTEL_INFO(dev)->gen >= 6) {
		dev_priv->gt_irq_mask &= ~(GT_GEN6_BSD_WATCHDOG_INTERRUPT |
					GT_GEN6_RENDER_WATCHDOG_INTERRUPT);
		gt_irqs |= GT_GEN6_RENDER_WATCHDOG_INTERRUPT;
		gt_irqs |= GT_GEN6_BSD_WATCHDOG_INTERRUPT;
	}
	if (IS_GEN5(dev)) {
		gt_irqs |= GT_RENDER_PIPECTL_NOTIFY_INTERRUPT |
			   ILK_BSD_USER_INTERRUPT;
	} else {
		gt_irqs |= GT_BLT_USER_INTERRUPT | GT_BSD_USER_INTERRUPT;
	}

	GEN5_IRQ_INIT(GT, dev_priv->gt_irq_mask, gt_irqs);

	if (INTEL_INFO(dev)->gen >= 6) {
		pm_irqs |= dev_priv->pm_rps_events;

		if (HAS_VEBOX(dev))
			pm_irqs |= PM_VEBOX_USER_INTERRUPT;

		dev_priv->pm_irq_mask = 0xffffffff;

		if (dev_priv->rps.use_RC0_residency_for_turbo) {
			dev_priv->pm_irq_mask &= ~GEN6_PM_RP_UP_EI_EXPIRED;
			pm_irqs &= ~GEN6_PM_RPS_EVENTS;
			pm_irqs |= GEN6_PM_RP_UP_EI_EXPIRED;
		} else {
			dev_priv->pm_irq_mask &= ~GEN6_PM_RPS_EVENTS;
			pm_irqs |= GEN6_PM_RPS_EVENTS;
		}

		GEN5_IRQ_INIT(GEN6_PM, dev_priv->pm_irq_mask, pm_irqs);
	}
}

static int ironlake_irq_postinstall(struct drm_device *dev)
{
	unsigned long irqflags;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 display_mask, extra_mask;

	if (INTEL_INFO(dev)->gen >= 7) {
		display_mask = (DE_MASTER_IRQ_CONTROL | DE_GSE_IVB |
				DE_PCH_EVENT_IVB | DE_PLANEC_FLIP_DONE_IVB |
				DE_PLANEB_FLIP_DONE_IVB |
				DE_PLANEA_FLIP_DONE_IVB | DE_AUX_CHANNEL_A_IVB);
		extra_mask = (DE_PIPEC_VBLANK_IVB | DE_PIPEB_VBLANK_IVB |
			      DE_PIPEA_VBLANK_IVB | DE_ERR_INT_IVB);

		if (I915_HAS_DPST(dev))
			display_mask |= DE_DPST_HISTOGRAM_IVB;

	} else {
		display_mask = (DE_MASTER_IRQ_CONTROL | DE_GSE | DE_PCH_EVENT |
				DE_PLANEA_FLIP_DONE | DE_PLANEB_FLIP_DONE |
				DE_AUX_CHANNEL_A |
				DE_PIPEB_CRC_DONE | DE_PIPEA_CRC_DONE |
				DE_POISON);
		extra_mask = DE_PIPEA_VBLANK | DE_PIPEB_VBLANK | DE_PCU_EVENT |
				DE_PIPEB_FIFO_UNDERRUN | DE_PIPEA_FIFO_UNDERRUN;
	}

	dev_priv->irq_mask = ~display_mask;

	I915_WRITE(HWSTAM, 0xeffe);

	ibx_irq_pre_postinstall(dev);

	GEN5_IRQ_INIT(DE, dev_priv->irq_mask, display_mask | extra_mask);

	gen5_gt_irq_postinstall(dev);

	ibx_irq_postinstall(dev);

	if (IS_IRONLAKE_M(dev)) {
		/* Enable PCU event interrupts
		 *
		 * spinlocking not required here for correctness since interrupt
		 * setup is guaranteed to run in single-threaded context. But we
		 * need it to make the assert_spin_locked happy. */
		spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
		ironlake_enable_display_irq(dev_priv, DE_PCU_EVENT);
		spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);
	}

	return 0;
}

static void valleyview_display_irqs_install(struct drm_i915_private *dev_priv)
{
	struct drm_device *dev = dev_priv->dev;
	u32 pipestat_mask;
	u32 iir_mask;
	int pipe;
#ifdef CONFIG_SUPPORT_LPDMA_HDMI_AUDIO
	u32 lpe_status_clear;
#endif

	pipestat_mask = PIPESTAT_INT_STATUS_MASK |
			PIPE_FIFO_UNDERRUN_STATUS;

	for_each_pipe(pipe)
		I915_WRITE(PIPESTAT(pipe), pipestat_mask);
	POSTING_READ(PIPESTAT(PIPE_A));

	/* Enable the DPST interrupt also */
	pipestat_mask = PLANE_FLIP_DONE_INT_STATUS_VLV |
			PIPE_CRC_DONE_INTERRUPT_STATUS |
			SPRITE0_FLIP_DONE_INT_STATUS_VLV |
			SPRITE1_FLIP_DONE_INT_STATUS_VLV |
			PIPE_DPST_EVENT_STATUS;

	i915_enable_pipestat(dev_priv, PIPE_A, PIPE_GMBUS_INTERRUPT_STATUS);
	for_each_pipe(pipe)
		i915_enable_pipestat(dev_priv, pipe, pipestat_mask);

	iir_mask = I915_DISPLAY_PORT_INTERRUPT |
			I915_DISPLAY_PIPE_A_EVENT_INTERRUPT |
			I915_DISPLAY_PIPE_B_EVENT_INTERRUPT;

	if (IS_CHERRYVIEW(dev))
		iir_mask |= I915_DISPLAY_PIPE_C_EVENT_INTERRUPT;
#ifdef CONFIG_SUPPORT_LPDMA_HDMI_AUDIO
	iir_mask |=	(I915_LPE_PIPE_A_INTERRUPT |
			I915_LPE_PIPE_B_INTERRUPT);

	if (IS_CHERRYVIEW(dev))
		iir_mask |= I915_LPE_PIPE_C_INTERRUPT;
#endif
	dev_priv->irq_mask &= ~iir_mask;

	I915_WRITE(VLV_IIR, iir_mask);
	I915_WRITE(VLV_IIR, iir_mask);
	I915_WRITE(VLV_IMR, dev_priv->irq_mask);
	I915_WRITE(VLV_IER, ~dev_priv->irq_mask);
#ifdef CONFIG_SUPPORT_LPDMA_HDMI_AUDIO
	lpe_status_clear = I915_HDMI_AUDIO_UNDERRUN |
			I915_HDMI_AUDIO_BUFFER_DONE;
	I915_WRITE(I915_LPE_AUDIO_HDMI_STATUS_A, lpe_status_clear);
	I915_WRITE(I915_LPE_AUDIO_HDMI_STATUS_B, lpe_status_clear);

	if (IS_CHERRYVIEW(dev))
		I915_WRITE(I915_LPE_AUDIO_HDMI_STATUS_C, lpe_status_clear);
#endif

	POSTING_READ(VLV_IER);
}

static void valleyview_display_irqs_uninstall(struct drm_i915_private *dev_priv)
{
	struct drm_device *dev = dev_priv->dev;
	u32 pipestat_mask;
	u32 iir_mask;
	int pipe;

	iir_mask = I915_DISPLAY_PORT_INTERRUPT |
		   I915_DISPLAY_PIPE_A_EVENT_INTERRUPT |
		   I915_DISPLAY_PIPE_B_EVENT_INTERRUPT;

	if (IS_CHERRYVIEW(dev))
		iir_mask |= I915_DISPLAY_PIPE_C_EVENT_INTERRUPT;

	dev_priv->irq_mask |= iir_mask;
	I915_WRITE(VLV_IER, ~dev_priv->irq_mask);
	I915_WRITE(VLV_IMR, dev_priv->irq_mask);
	I915_WRITE(VLV_IIR, iir_mask);
	I915_WRITE(VLV_IIR, iir_mask);
	POSTING_READ(VLV_IIR);

	pipestat_mask = PLANE_FLIP_DONE_INT_STATUS_VLV |
			PIPE_CRC_DONE_INTERRUPT_STATUS |
			PIPE_DPST_EVENT_STATUS;

	i915_disable_pipestat(dev_priv, PIPE_A, PIPE_GMBUS_INTERRUPT_STATUS);
	for_each_pipe(pipe)
		i915_disable_pipestat(dev_priv, pipe, pipestat_mask);

	pipestat_mask = PIPESTAT_INT_STATUS_MASK |
			PIPE_FIFO_UNDERRUN_STATUS;

	for_each_pipe(pipe)
		I915_WRITE(PIPESTAT(pipe), pipestat_mask);
	POSTING_READ(PIPESTAT(PIPE_A));
}

void valleyview_enable_display_irqs(struct drm_i915_private *dev_priv)
{
	assert_spin_locked(&dev_priv->irq_lock);

	if (dev_priv->display_irqs_enabled)
		return;

	dev_priv->display_irqs_enabled = true;

	if (dev_priv->dev->irq_enabled)
		valleyview_display_irqs_install(dev_priv);
}

void valleyview_disable_display_irqs(struct drm_i915_private *dev_priv)
{
	assert_spin_locked(&dev_priv->irq_lock);

	if (!dev_priv->display_irqs_enabled)
		return;

	dev_priv->display_irqs_enabled = false;

	if (dev_priv->dev->irq_enabled)
		valleyview_display_irqs_uninstall(dev_priv);
}

static int valleyview_irq_postinstall(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long irqflags;

	dev_priv->irq_mask = ~0;

	I915_WRITE(PORT_HOTPLUG_EN, 0);
	POSTING_READ(PORT_HOTPLUG_EN);

	I915_WRITE(VLV_IMR, dev_priv->irq_mask);
	I915_WRITE(VLV_IER, ~dev_priv->irq_mask);
	I915_WRITE(VLV_IIR, 0xffffffff);
	POSTING_READ(VLV_IER);

	/* Interrupt setup is already guaranteed to be single-threaded, this is
	 * just to make the assert_spin_locked check happy. */
	spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
	if (dev_priv->display_irqs_enabled)
		valleyview_display_irqs_install(dev_priv);
	spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);

	I915_WRITE(VLV_IIR, 0xffffffff);
	I915_WRITE(VLV_IIR, 0xffffffff);

	gen5_gt_irq_postinstall(dev);

	/* ack & enable invalid PTE error interrupts */
#if 0 /* FIXME: add support to irq handler for checking these bits */
	I915_WRITE(DPINVGTT, DPINVGTT_STATUS_MASK);
	I915_WRITE(DPINVGTT, DPINVGTT_EN_MASK);
#endif

	I915_WRITE(VLV_MASTER_IER, MASTER_INTERRUPT_ENABLE);

	return 0;
}

static void gen8_gt_irq_postinstall(struct drm_i915_private *dev_priv)
{
	/* These are interrupts we'll toggle with the ring mask register */
	uint32_t gt_interrupts[] = {
		GT_GEN8_RCS_WATCHDOG_INTERRUPT << GEN8_RCS_IRQ_SHIFT |
		GT_RENDER_USER_INTERRUPT << GEN8_RCS_IRQ_SHIFT |
			GT_CONTEXT_SWITCH_INTERRUPT << GEN8_RCS_IRQ_SHIFT |
			GT_RENDER_L3_PARITY_ERROR_INTERRUPT |
			GT_RENDER_USER_INTERRUPT << GEN8_BCS_IRQ_SHIFT |
			GT_CONTEXT_SWITCH_INTERRUPT << GEN8_BCS_IRQ_SHIFT,
		GT_GEN8_VCS_WATCHDOG_INTERRUPT << GEN8_VCS1_IRQ_SHIFT |
		GT_GEN8_VCS_WATCHDOG_INTERRUPT << GEN8_VCS2_IRQ_SHIFT |
		GT_RENDER_USER_INTERRUPT << GEN8_VCS1_IRQ_SHIFT |
			GT_CONTEXT_SWITCH_INTERRUPT << GEN8_VCS1_IRQ_SHIFT |
			GT_RENDER_USER_INTERRUPT << GEN8_VCS2_IRQ_SHIFT |
			GT_CONTEXT_SWITCH_INTERRUPT << GEN8_VCS2_IRQ_SHIFT,
		0,
		GT_RENDER_USER_INTERRUPT << GEN8_VECS_IRQ_SHIFT |
			GT_CONTEXT_SWITCH_INTERRUPT << GEN8_VECS_IRQ_SHIFT |
			GT_RENDER_PERFMON_BUFFER_INTERRUPT <<
				GEN8_OACS_IRQ_SHIFT
		};

	dev_priv->pm_irq_mask = 0xffffffff;
	GEN8_IRQ_INIT_NDX(GT, 0, ~gt_interrupts[0], gt_interrupts[0]);
	GEN8_IRQ_INIT_NDX(GT, 1, ~gt_interrupts[1], gt_interrupts[1]);
	GEN8_IRQ_INIT_NDX(GT, 2, dev_priv->pm_irq_mask, dev_priv->pm_rps_events);
	GEN8_IRQ_INIT_NDX(GT, 3, ~gt_interrupts[3], gt_interrupts[3]);
}

static void gen8_de_irq_postinstall(struct drm_i915_private *dev_priv)
{
	struct drm_device *dev = dev_priv->dev;
	uint32_t de_pipe_masked = GEN8_PIPE_PRIMARY_FLIP_DONE |
		GEN8_PIPE_CDCLK_CRC_DONE |
		GEN8_DE_PIPE_IRQ_FAULT_ERRORS;
	uint32_t de_pipe_enables;
	int pipe;

	if (I915_HAS_DPST(dev))
		de_pipe_masked |= GEN8_PIPE_DPST_INTERRUPT;

	de_pipe_enables = de_pipe_masked | GEN8_PIPE_VBLANK |
					GEN8_PIPE_FIFO_UNDERRUN;

	dev_priv->de_irq_mask[PIPE_A] = ~de_pipe_masked;
	dev_priv->de_irq_mask[PIPE_B] = ~de_pipe_masked;
	dev_priv->de_irq_mask[PIPE_C] = ~de_pipe_masked;

	for_each_pipe(pipe)
		GEN8_IRQ_INIT_NDX(DE_PIPE, pipe, dev_priv->de_irq_mask[pipe],
				  de_pipe_enables);

	GEN5_IRQ_INIT(GEN8_DE_PORT_, ~GEN8_AUX_CHANNEL_A, GEN8_AUX_CHANNEL_A);
}

static int gen8_irq_postinstall(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	ibx_irq_pre_postinstall(dev);

	gen8_gt_irq_postinstall(dev_priv);
	gen8_de_irq_postinstall(dev_priv);

	ibx_irq_postinstall(dev);

	I915_WRITE(GEN8_MASTER_IRQ, DE_MASTER_IRQ_CONTROL);
	POSTING_READ(GEN8_MASTER_IRQ);

	return 0;
}

static int cherryview_irq_postinstall(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 enable_mask = I915_DISPLAY_PORT_INTERRUPT |
		I915_DISPLAY_PIPE_A_EVENT_INTERRUPT |
		I915_DISPLAY_PIPE_B_EVENT_INTERRUPT |
		I915_DISPLAY_PIPE_C_EVENT_INTERRUPT |
		I915_LPE_PIPE_C_INTERRUPT |
		CHV_I915_LPE_PIPE_A_INTERRUPT |
		CHV_I915_LPE_PIPE_B_INTERRUPT;
	u32 pipestat_enable = PLANE_FLIP_DONE_INT_STATUS_VLV |
		SPRITE0_FLIP_DONE_INT_STATUS_VLV |
		SPRITE1_FLIP_DONE_INT_STATUS_VLV |
		PIPE_CRC_DONE_INTERRUPT_STATUS |
		PIPE_DPST_EVENT_STATUS |
		I915_LPE_PIPE_C_INTERRUPT;
	unsigned long irqflags;
	u32 lpe_status_clear;
	int pipe;

	/*
	 * Leave vblank interrupts masked initially.  enable/disable will
	 * toggle them based on usage.
	 */
	dev_priv->irq_mask = ~enable_mask;

	for_each_pipe(pipe)
		I915_WRITE(PIPESTAT(pipe), 0xffff);

	/* Added for HDMI Audio */
	lpe_status_clear = I915_HDMI_AUDIO_UNDERRUN |
	I915_HDMI_AUDIO_BUFFER_DONE;
	I915_WRITE(I915_LPE_AUDIO_HDMI_STATUS_A, lpe_status_clear);
	I915_WRITE(I915_LPE_AUDIO_HDMI_STATUS_B, lpe_status_clear);
	I915_WRITE(I915_LPE_AUDIO_HDMI_STATUS_C, lpe_status_clear);
	POSTING_READ(VLV_IER);

	spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
	i915_enable_pipestat(dev_priv, PIPE_A, PIPE_GMBUS_INTERRUPT_STATUS);
	for_each_pipe(pipe)
		i915_enable_pipestat(dev_priv, pipe, pipestat_enable);
	spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);

	I915_WRITE(VLV_IIR, 0xffffffff);
	I915_WRITE(VLV_IMR, dev_priv->irq_mask);
	I915_WRITE(VLV_IER, enable_mask);

	gen8_gt_irq_postinstall(dev_priv);

	I915_WRITE(GEN8_MASTER_IRQ, MASTER_INTERRUPT_ENABLE);
	POSTING_READ(GEN8_MASTER_IRQ);

	return 0;
}

static void gen8_irq_uninstall(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (!dev_priv)
		return;

	intel_hpd_irq_uninstall(dev_priv);

	gen8_irq_reset(dev);
}

static void valleyview_irq_uninstall(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long irqflags;
	int pipe;

	if (!dev_priv)
		return;

	I915_WRITE(VLV_MASTER_IER, 0);

	intel_hpd_irq_uninstall(dev_priv);

	for_each_pipe(pipe)
		I915_WRITE(PIPESTAT(pipe), 0xffff);

	I915_WRITE(HWSTAM, 0xffffffff);
	I915_WRITE(PORT_HOTPLUG_EN, 0);
	I915_WRITE(PORT_HOTPLUG_STAT, I915_READ(PORT_HOTPLUG_STAT));

	spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
	if (dev_priv->display_irqs_enabled)
		valleyview_display_irqs_uninstall(dev_priv);
	spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);

	dev_priv->irq_mask = 0;

	I915_WRITE(VLV_IIR, 0xffffffff);
	I915_WRITE(VLV_IMR, 0xffffffff);
	I915_WRITE(VLV_IER, 0x0);
	POSTING_READ(VLV_IER);
}

static void cherryview_irq_uninstall(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int pipe;

	if (!dev_priv)
		return;

	I915_WRITE(GEN8_MASTER_IRQ, 0);
	POSTING_READ(GEN8_MASTER_IRQ);

#define GEN8_IRQ_FINI_NDX(type, which)				\
do {								\
	I915_WRITE(GEN8_##type##_IMR(which), 0xffffffff);	\
	I915_WRITE(GEN8_##type##_IER(which), 0);		\
	I915_WRITE(GEN8_##type##_IIR(which), 0xffffffff);	\
	POSTING_READ(GEN8_##type##_IIR(which));			\
	I915_WRITE(GEN8_##type##_IIR(which), 0xffffffff);	\
} while (0)

#define GEN8_IRQ_FINI(type)				\
do {							\
	I915_WRITE(GEN8_##type##_IMR, 0xffffffff);	\
	I915_WRITE(GEN8_##type##_IER, 0);		\
	I915_WRITE(GEN8_##type##_IIR, 0xffffffff);	\
	POSTING_READ(GEN8_##type##_IIR);		\
	I915_WRITE(GEN8_##type##_IIR, 0xffffffff);	\
} while (0)

	GEN8_IRQ_FINI_NDX(GT, 0);
	GEN8_IRQ_FINI_NDX(GT, 1);
	GEN8_IRQ_FINI_NDX(GT, 2);
	GEN8_IRQ_FINI_NDX(GT, 3);

	GEN8_IRQ_FINI(PCU);

#undef GEN8_IRQ_FINI
#undef GEN8_IRQ_FINI_NDX

	I915_WRITE(PORT_HOTPLUG_EN, 0);
	I915_WRITE(PORT_HOTPLUG_STAT, I915_READ(PORT_HOTPLUG_STAT));

	for_each_pipe(pipe)
		I915_WRITE(PIPESTAT(pipe), 0xffff);

	I915_WRITE(VLV_IMR, 0xffffffff);
	I915_WRITE(VLV_IER, 0x0);
	I915_WRITE(VLV_IIR, 0xffffffff);
	POSTING_READ(VLV_IIR);
}

static void ironlake_irq_uninstall(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (!dev_priv)
		return;

	intel_hpd_irq_uninstall(dev_priv);

	ironlake_irq_reset(dev);
}

static void i8xx_irq_preinstall(struct drm_device * dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int pipe;

	for_each_pipe(pipe)
		I915_WRITE(PIPESTAT(pipe), 0);
	I915_WRITE16(IMR, 0xffff);
	I915_WRITE16(IER, 0x0);
	POSTING_READ16(IER);
}

static int i8xx_irq_postinstall(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long irqflags;

	I915_WRITE16(EMR,
		     ~(I915_ERROR_PAGE_TABLE | I915_ERROR_MEMORY_REFRESH));

	/* Unmask the interrupts that we always want on. */
	dev_priv->irq_mask =
		~(I915_DISPLAY_PIPE_A_EVENT_INTERRUPT |
		  I915_DISPLAY_PIPE_B_EVENT_INTERRUPT |
		  I915_DISPLAY_PLANE_A_FLIP_PENDING_INTERRUPT |
		  I915_DISPLAY_PLANE_B_FLIP_PENDING_INTERRUPT |
		  I915_RENDER_COMMAND_PARSER_ERROR_INTERRUPT);
	I915_WRITE16(IMR, dev_priv->irq_mask);

	I915_WRITE16(IER,
		     I915_DISPLAY_PIPE_A_EVENT_INTERRUPT |
		     I915_DISPLAY_PIPE_B_EVENT_INTERRUPT |
		     I915_RENDER_COMMAND_PARSER_ERROR_INTERRUPT |
		     I915_USER_INTERRUPT);
	POSTING_READ16(IER);

	/* Interrupt setup is already guaranteed to be single-threaded, this is
	 * just to make the assert_spin_locked check happy. */
	spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
	i915_enable_pipestat(dev_priv, PIPE_A, PIPE_CRC_DONE_INTERRUPT_STATUS);
	i915_enable_pipestat(dev_priv, PIPE_B, PIPE_CRC_DONE_INTERRUPT_STATUS);
	spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);

	return 0;
}

/*
 * Returns true when a page flip has completed.
 */
static bool i8xx_handle_vblank(struct drm_device *dev,
			       int plane, int pipe, u32 iir)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u16 flip_pending = DISPLAY_PLANE_FLIP_PENDING(plane);

	if (!intel_pipe_handle_vblank(dev, pipe))
		return false;

	if ((iir & flip_pending) == 0)
		return false;

	intel_prepare_page_flip(dev, plane);

	/* We detect FlipDone by looking for the change in PendingFlip from '1'
	 * to '0' on the following vblank, i.e. IIR has the Pendingflip
	 * asserted following the MI_DISPLAY_FLIP, but ISR is deasserted, hence
	 * the flip is completed (no longer pending). Since this doesn't raise
	 * an interrupt per se, we watch for the change at vblank.
	 */
	if (I915_READ16(ISR) & flip_pending)
		return false;

	intel_finish_page_flip(dev, pipe);

	return true;
}

static irqreturn_t i8xx_irq_handler(int irq, void *arg)
{
	struct drm_device *dev = arg;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u16 iir, new_iir;
	u32 pipe_stats[2];
	unsigned long irqflags;
	int pipe;
	u16 flip_mask =
		I915_DISPLAY_PLANE_A_FLIP_PENDING_INTERRUPT |
		I915_DISPLAY_PLANE_B_FLIP_PENDING_INTERRUPT;

	iir = I915_READ16(IIR);
	if (iir == 0)
		return IRQ_NONE;

	while (iir & ~flip_mask) {
		/* Can't rely on pipestat interrupt bit in iir as it might
		 * have been cleared after the pipestat interrupt was received.
		 * It doesn't set the bit in iir again, but it still produces
		 * interrupts (for non-MSI).
		 */
		spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
		if (iir & I915_RENDER_COMMAND_PARSER_ERROR_INTERRUPT)
			i915_handle_error(dev, NULL, 0,
					  "Command parser error, iir 0x%08x",
					  iir);

		for_each_pipe(pipe) {
			int reg = PIPESTAT(pipe);
			pipe_stats[pipe] = I915_READ(reg);

			/*
			 * Clear the PIPE*STAT regs before the IIR
			 */
			if (pipe_stats[pipe] & 0x8000ffff)
				I915_WRITE(reg, pipe_stats[pipe]);
		}
		spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);

		I915_WRITE16(IIR, iir & ~flip_mask);
		new_iir = I915_READ16(IIR); /* Flush posted writes */

		i915_update_dri1_breadcrumb(dev);

		if (iir & I915_USER_INTERRUPT)
			notify_ring(dev, &dev_priv->ring[RCS]);

		for_each_pipe(pipe) {
			int plane = pipe;
			if (HAS_FBC(dev))
				plane = !plane;

			if (pipe_stats[pipe] & PIPE_VBLANK_INTERRUPT_STATUS &&
			    i8xx_handle_vblank(dev, plane, pipe, iir))
				flip_mask &= ~DISPLAY_PLANE_FLIP_PENDING(plane);

			if (pipe_stats[pipe] & PIPE_CRC_DONE_INTERRUPT_STATUS)
				i9xx_pipe_crc_irq_handler(dev, pipe);

			if (pipe_stats[pipe] & PIPE_FIFO_UNDERRUN_STATUS &&
			    intel_set_cpu_fifo_underrun_reporting(dev, pipe, false))
				DRM_ERROR("pipe %c underrun\n", pipe_name(pipe));
		}

		iir = new_iir;
	}

	return IRQ_HANDLED;
}

static void i8xx_irq_uninstall(struct drm_device * dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int pipe;

	for_each_pipe(pipe) {
		/* Clear enable bits; then clear status bits */
		I915_WRITE(PIPESTAT(pipe), 0);
		I915_WRITE(PIPESTAT(pipe), I915_READ(PIPESTAT(pipe)));
	}
	I915_WRITE16(IMR, 0xffff);
	I915_WRITE16(IER, 0x0);
	I915_WRITE16(IIR, I915_READ16(IIR));
}

static void i915_irq_preinstall(struct drm_device * dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int pipe;

	if (I915_HAS_HOTPLUG(dev)) {
		I915_WRITE(PORT_HOTPLUG_EN, 0);
		I915_WRITE(PORT_HOTPLUG_STAT, I915_READ(PORT_HOTPLUG_STAT));
	}

	I915_WRITE16(HWSTAM, 0xeffe);
	for_each_pipe(pipe)
		I915_WRITE(PIPESTAT(pipe), 0);
	I915_WRITE(IMR, 0xffffffff);
	I915_WRITE(IER, 0x0);
	POSTING_READ(IER);
}

static int i915_irq_postinstall(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 enable_mask;
	unsigned long irqflags;

	I915_WRITE(EMR, ~(I915_ERROR_PAGE_TABLE | I915_ERROR_MEMORY_REFRESH));

	/* Unmask the interrupts that we always want on. */
	dev_priv->irq_mask =
		~(I915_ASLE_INTERRUPT |
		  I915_DISPLAY_PIPE_A_EVENT_INTERRUPT |
		  I915_DISPLAY_PIPE_B_EVENT_INTERRUPT |
		  I915_DISPLAY_PLANE_A_FLIP_PENDING_INTERRUPT |
		  I915_DISPLAY_PLANE_B_FLIP_PENDING_INTERRUPT |
		  I915_RENDER_COMMAND_PARSER_ERROR_INTERRUPT);

	enable_mask =
		I915_ASLE_INTERRUPT |
		I915_DISPLAY_PIPE_A_EVENT_INTERRUPT |
		I915_DISPLAY_PIPE_B_EVENT_INTERRUPT |
		I915_RENDER_COMMAND_PARSER_ERROR_INTERRUPT |
		I915_USER_INTERRUPT;

	if (I915_HAS_HOTPLUG(dev)) {
		I915_WRITE(PORT_HOTPLUG_EN, 0);
		POSTING_READ(PORT_HOTPLUG_EN);

		/* Enable in IER... */
		enable_mask |= I915_DISPLAY_PORT_INTERRUPT;
		/* and unmask in IMR */
		dev_priv->irq_mask &= ~I915_DISPLAY_PORT_INTERRUPT;
	}

	I915_WRITE(IMR, dev_priv->irq_mask);
	I915_WRITE(IER, enable_mask);
	POSTING_READ(IER);

	i915_enable_asle_pipestat(dev);

	/* Interrupt setup is already guaranteed to be single-threaded, this is
	 * just to make the assert_spin_locked check happy. */
	spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
	i915_enable_pipestat(dev_priv, PIPE_A, PIPE_CRC_DONE_INTERRUPT_STATUS);
	i915_enable_pipestat(dev_priv, PIPE_B, PIPE_CRC_DONE_INTERRUPT_STATUS);
	spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);

	return 0;
}

/*
 * Returns true when a page flip has completed.
 */
static bool i915_handle_vblank(struct drm_device *dev,
			       int plane, int pipe, u32 iir)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 flip_pending = DISPLAY_PLANE_FLIP_PENDING(plane);

	if (!intel_pipe_handle_vblank(dev, pipe))
		return false;

	if ((iir & flip_pending) == 0)
		return false;

	intel_prepare_page_flip(dev, plane);

	/* We detect FlipDone by looking for the change in PendingFlip from '1'
	 * to '0' on the following vblank, i.e. IIR has the Pendingflip
	 * asserted following the MI_DISPLAY_FLIP, but ISR is deasserted, hence
	 * the flip is completed (no longer pending). Since this doesn't raise
	 * an interrupt per se, we watch for the change at vblank.
	 */
	if (I915_READ(ISR) & flip_pending)
		return false;

	intel_finish_page_flip(dev, pipe);

	return true;
}

static irqreturn_t i915_irq_handler(int irq, void *arg)
{
	struct drm_device *dev = arg;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 iir, new_iir, pipe_stats[I915_MAX_PIPES] = {0};
	unsigned long irqflags;
	u32 flip_mask =
		I915_DISPLAY_PLANE_A_FLIP_PENDING_INTERRUPT |
		I915_DISPLAY_PLANE_B_FLIP_PENDING_INTERRUPT;
	int pipe, ret = IRQ_NONE;

	iir = I915_READ(IIR);
	do {
		bool irq_received = (iir & ~flip_mask) != 0;
		bool blc_event = false;

		/* Can't rely on pipestat interrupt bit in iir as it might
		 * have been cleared after the pipestat interrupt was received.
		 * It doesn't set the bit in iir again, but it still produces
		 * interrupts (for non-MSI).
		 */
		spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
		if (iir & I915_RENDER_COMMAND_PARSER_ERROR_INTERRUPT)
			i915_handle_error(dev, NULL, 0,
					  "Command parser error, iir 0x%08x",
					  iir);

		for_each_pipe(pipe) {
			int reg = PIPESTAT(pipe);
			pipe_stats[pipe] = I915_READ(reg);

			/* Clear the PIPE*STAT regs before the IIR */
			if (pipe_stats[pipe] & 0x8000ffff) {
				I915_WRITE(reg, pipe_stats[pipe]);
				irq_received = true;
			}
		}
		spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);

		if (!irq_received)
			break;

		/* Consume port.  Then clear IIR or we'll miss events */
		if (I915_HAS_HOTPLUG(dev) &&
		    iir & I915_DISPLAY_PORT_INTERRUPT)
			i9xx_hpd_irq_handler(dev);

		I915_WRITE(IIR, iir & ~flip_mask);
		new_iir = I915_READ(IIR); /* Flush posted writes */

		if (iir & I915_USER_INTERRUPT)
			notify_ring(dev, &dev_priv->ring[RCS]);

		for_each_pipe(pipe) {
			int plane = pipe;
			if (HAS_FBC(dev))
				plane = !plane;

			if (pipe_stats[pipe] & PIPE_VBLANK_INTERRUPT_STATUS &&
			    i915_handle_vblank(dev, plane, pipe, iir))
				flip_mask &= ~DISPLAY_PLANE_FLIP_PENDING(plane);

			if (pipe_stats[pipe] & PIPE_LEGACY_BLC_EVENT_STATUS)
				blc_event = true;

			if (pipe_stats[pipe] & PIPE_CRC_DONE_INTERRUPT_STATUS)
				i9xx_pipe_crc_irq_handler(dev, pipe);

			if (pipe_stats[pipe] & PIPE_FIFO_UNDERRUN_STATUS &&
			    intel_set_cpu_fifo_underrun_reporting(dev, pipe, false))
				DRM_ERROR("pipe %c underrun\n", pipe_name(pipe));
		}

		if (blc_event || (iir & I915_ASLE_INTERRUPT))
			intel_opregion_asle_intr(dev);

		/* With MSI, interrupts are only generated when iir
		 * transitions from zero to nonzero.  If another bit got
		 * set while we were handling the existing iir bits, then
		 * we would never get another interrupt.
		 *
		 * This is fine on non-MSI as well, as if we hit this path
		 * we avoid exiting the interrupt handler only to generate
		 * another one.
		 *
		 * Note that for MSI this could cause a stray interrupt report
		 * if an interrupt landed in the time between writing IIR and
		 * the posting read.  This should be rare enough to never
		 * trigger the 99% of 100,000 interrupts test for disabling
		 * stray interrupts.
		 */
		ret = IRQ_HANDLED;
		iir = new_iir;
	} while (iir & ~flip_mask);

	i915_update_dri1_breadcrumb(dev);

	return ret;
}

static void i915_irq_uninstall(struct drm_device * dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int pipe;

	intel_hpd_irq_uninstall(dev_priv);

	if (I915_HAS_HOTPLUG(dev)) {
		I915_WRITE(PORT_HOTPLUG_EN, 0);
		I915_WRITE(PORT_HOTPLUG_STAT, I915_READ(PORT_HOTPLUG_STAT));
	}

	I915_WRITE16(HWSTAM, 0xffff);
	for_each_pipe(pipe) {
		/* Clear enable bits; then clear status bits */
		I915_WRITE(PIPESTAT(pipe), 0);
		I915_WRITE(PIPESTAT(pipe), I915_READ(PIPESTAT(pipe)));
	}
	I915_WRITE(IMR, 0xffffffff);
	I915_WRITE(IER, 0x0);

	I915_WRITE(IIR, I915_READ(IIR));
}

static void i965_irq_preinstall(struct drm_device * dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int pipe;

	I915_WRITE(PORT_HOTPLUG_EN, 0);
	I915_WRITE(PORT_HOTPLUG_STAT, I915_READ(PORT_HOTPLUG_STAT));

	I915_WRITE(HWSTAM, 0xeffe);
	for_each_pipe(pipe)
		I915_WRITE(PIPESTAT(pipe), 0);
	I915_WRITE(IMR, 0xffffffff);
	I915_WRITE(IER, 0x0);
	POSTING_READ(IER);
}

static int i965_irq_postinstall(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 enable_mask;
	u32 error_mask;
	unsigned long irqflags;

	/* Unmask the interrupts that we always want on. */
	dev_priv->irq_mask = ~(I915_ASLE_INTERRUPT |
			       I915_DISPLAY_PORT_INTERRUPT |
			       I915_DISPLAY_PIPE_A_EVENT_INTERRUPT |
			       I915_DISPLAY_PIPE_B_EVENT_INTERRUPT |
			       I915_DISPLAY_PLANE_A_FLIP_PENDING_INTERRUPT |
			       I915_DISPLAY_PLANE_B_FLIP_PENDING_INTERRUPT |
			       I915_RENDER_COMMAND_PARSER_ERROR_INTERRUPT);

	enable_mask = ~dev_priv->irq_mask;
	enable_mask &= ~(I915_DISPLAY_PLANE_A_FLIP_PENDING_INTERRUPT |
			 I915_DISPLAY_PLANE_B_FLIP_PENDING_INTERRUPT);
	enable_mask |= I915_USER_INTERRUPT;

	if (IS_G4X(dev))
		enable_mask |= I915_BSD_USER_INTERRUPT;

	/* Interrupt setup is already guaranteed to be single-threaded, this is
	 * just to make the assert_spin_locked check happy. */
	spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
	i915_enable_pipestat(dev_priv, PIPE_A, PIPE_GMBUS_INTERRUPT_STATUS);
	i915_enable_pipestat(dev_priv, PIPE_A, PIPE_CRC_DONE_INTERRUPT_STATUS);
	i915_enable_pipestat(dev_priv, PIPE_B, PIPE_CRC_DONE_INTERRUPT_STATUS);
	spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);

	/*
	 * Enable some error detection, note the instruction error mask
	 * bit is reserved, so we leave it masked.
	 */
	if (IS_G4X(dev)) {
		error_mask = ~(GM45_ERROR_PAGE_TABLE |
			       GM45_ERROR_MEM_PRIV |
			       GM45_ERROR_CP_PRIV |
			       I915_ERROR_MEMORY_REFRESH);
	} else {
		error_mask = ~(I915_ERROR_PAGE_TABLE |
			       I915_ERROR_MEMORY_REFRESH);
	}
	I915_WRITE(EMR, error_mask);

	I915_WRITE(IMR, dev_priv->irq_mask);
	I915_WRITE(IER, enable_mask);
	POSTING_READ(IER);

	I915_WRITE(PORT_HOTPLUG_EN, 0);
	POSTING_READ(PORT_HOTPLUG_EN);

	i915_enable_asle_pipestat(dev);

	return 0;
}

static void i915_hpd_irq_setup(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_mode_config *mode_config = &dev->mode_config;
	struct intel_encoder *intel_encoder;
	u32 hotplug_en;

	assert_spin_locked(&dev_priv->irq_lock);

	if (I915_HAS_HOTPLUG(dev)) {
		hotplug_en = I915_READ(PORT_HOTPLUG_EN);
		hotplug_en &= ~HOTPLUG_INT_EN_MASK;

		/*
		 * Note HDMI and DP share hotplug bits
		 * enable bits are the same for all generations
		 * Also, avoid enabling HPD for eDP since it is not required
		 */
		list_for_each_entry(intel_encoder,
			&mode_config->encoder_list, base.head) {
			int hpd_status;
			int port = intel_encoder->hpd_pin;
			hpd_status = dev_priv->hpd_stats[port].hpd_mark;
			if ((intel_encoder->type != INTEL_OUTPUT_EDP) &&
				(hpd_status == HPD_ENABLED))
				hotplug_en |= hpd_mask_i915[intel_encoder->hpd_pin];
		}
		/* Programming the CRT detection parameters tends
		   to generate a spurious hotplug event about three
		   seconds later.  So just do it once.
		*/
		if (IS_G4X(dev))
			hotplug_en |= CRT_HOTPLUG_ACTIVATION_PERIOD_64;
		hotplug_en &= ~CRT_HOTPLUG_VOLTAGE_COMPARE_MASK;
		hotplug_en |= CRT_HOTPLUG_VOLTAGE_COMPARE_50;

		/* Ignore TV since it's buggy */
		I915_WRITE(PORT_HOTPLUG_EN, hotplug_en);
	}
}

static irqreturn_t i965_irq_handler(int irq, void *arg)
{
	struct drm_device *dev = arg;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 iir, new_iir;
	u32 pipe_stats[I915_MAX_PIPES] = {0};
	unsigned long irqflags;
	int ret = IRQ_NONE, pipe;
	u32 flip_mask =
		I915_DISPLAY_PLANE_A_FLIP_PENDING_INTERRUPT |
		I915_DISPLAY_PLANE_B_FLIP_PENDING_INTERRUPT;

	iir = I915_READ(IIR);

	for (;;) {
		bool irq_received = (iir & ~flip_mask) != 0;
		bool blc_event = false;

		/* Can't rely on pipestat interrupt bit in iir as it might
		 * have been cleared after the pipestat interrupt was received.
		 * It doesn't set the bit in iir again, but it still produces
		 * interrupts (for non-MSI).
		 */
		spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
		if (iir & I915_RENDER_COMMAND_PARSER_ERROR_INTERRUPT)
			i915_handle_error(dev, NULL, 0,
					  "Command parser error, iir 0x%08x",
					  iir);

		for_each_pipe(pipe) {
			int reg = PIPESTAT(pipe);
			pipe_stats[pipe] = I915_READ(reg);

			/*
			 * Clear the PIPE*STAT regs before the IIR
			 */
			if (pipe_stats[pipe] & 0x8000ffff) {
				I915_WRITE(reg, pipe_stats[pipe]);
				irq_received = true;
			}
		}
		spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);

		if (!irq_received)
			break;

		ret = IRQ_HANDLED;

		/* Consume port.  Then clear IIR or we'll miss events */
		if (iir & I915_DISPLAY_PORT_INTERRUPT)
			i9xx_hpd_irq_handler(dev);

		I915_WRITE(IIR, iir & ~flip_mask);
		new_iir = I915_READ(IIR); /* Flush posted writes */

		if (iir & I915_USER_INTERRUPT)
			notify_ring(dev, &dev_priv->ring[RCS]);
		if (iir & I915_BSD_USER_INTERRUPT)
			notify_ring(dev, &dev_priv->ring[VCS]);

		for_each_pipe(pipe) {
			if (pipe_stats[pipe] & PIPE_START_VBLANK_INTERRUPT_STATUS &&
			    i915_handle_vblank(dev, pipe, pipe, iir))
				flip_mask &= ~DISPLAY_PLANE_FLIP_PENDING(pipe);

			if (pipe_stats[pipe] & PIPE_LEGACY_BLC_EVENT_STATUS)
				blc_event = true;

			if (pipe_stats[pipe] & PIPE_CRC_DONE_INTERRUPT_STATUS)
				i9xx_pipe_crc_irq_handler(dev, pipe);

			if (pipe_stats[pipe] & PIPE_FIFO_UNDERRUN_STATUS &&
			    intel_set_cpu_fifo_underrun_reporting(dev, pipe, false))
				DRM_ERROR("pipe %c underrun\n", pipe_name(pipe));
		}

		if (blc_event || (iir & I915_ASLE_INTERRUPT))
			intel_opregion_asle_intr(dev);

		if (pipe_stats[0] & PIPE_GMBUS_INTERRUPT_STATUS)
			gmbus_irq_handler(dev);

		/* With MSI, interrupts are only generated when iir
		 * transitions from zero to nonzero.  If another bit got
		 * set while we were handling the existing iir bits, then
		 * we would never get another interrupt.
		 *
		 * This is fine on non-MSI as well, as if we hit this path
		 * we avoid exiting the interrupt handler only to generate
		 * another one.
		 *
		 * Note that for MSI this could cause a stray interrupt report
		 * if an interrupt landed in the time between writing IIR and
		 * the posting read.  This should be rare enough to never
		 * trigger the 99% of 100,000 interrupts test for disabling
		 * stray interrupts.
		 */
		iir = new_iir;
	}

	i915_update_dri1_breadcrumb(dev);

	return ret;
}

static void i965_irq_uninstall(struct drm_device * dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int pipe;

	if (!dev_priv)
		return;

	intel_hpd_irq_uninstall(dev_priv);

	I915_WRITE(PORT_HOTPLUG_EN, 0);
	I915_WRITE(PORT_HOTPLUG_STAT, I915_READ(PORT_HOTPLUG_STAT));

	I915_WRITE(HWSTAM, 0xffffffff);
	for_each_pipe(pipe)
		I915_WRITE(PIPESTAT(pipe), 0);
	I915_WRITE(IMR, 0xffffffff);
	I915_WRITE(IER, 0x0);

	for_each_pipe(pipe)
		I915_WRITE(PIPESTAT(pipe),
			   I915_READ(PIPESTAT(pipe)) & 0x8000ffff);
	I915_WRITE(IIR, I915_READ(IIR));
}

static void intel_hpd_irq_reenable(unsigned long data)
{
	struct drm_i915_private *dev_priv = (struct drm_i915_private *)data;
	struct drm_device *dev = dev_priv->dev;
	struct drm_mode_config *mode_config = &dev->mode_config;
	unsigned long irqflags;
	int i;

	spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
	for (i = (HPD_NONE + 1); i < HPD_NUM_PINS; i++) {
		struct drm_connector *connector;

		if (dev_priv->hpd_stats[i].hpd_mark != HPD_DISABLED)
			continue;

		dev_priv->hpd_stats[i].hpd_mark = HPD_ENABLED;

		list_for_each_entry(connector, &mode_config->connector_list, head) {
			struct intel_connector *intel_connector = to_intel_connector(connector);

			if (intel_connector->encoder->hpd_pin == i) {
				if (connector->polled != intel_connector->polled)
					DRM_DEBUG_DRIVER("Reenabling HPD on connector %s\n",
							 connector->name);
				connector->polled = intel_connector->polled;
				if (!connector->polled)
					connector->polled = DRM_CONNECTOR_POLL_HPD;
			}
		}
	}
	if (dev_priv->display.hpd_irq_setup)
		dev_priv->display.hpd_irq_setup(dev);
	spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);
}

void intel_irq_init(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	INIT_WORK(&dev_priv->hotplug_work, i915_hotplug_work_func);
	INIT_WORK(&dev_priv->dig_port_work, i915_digport_work_func);
	INIT_WORK(&dev_priv->gpu_error.work, i915_error_work_func);
	INIT_WORK(&dev_priv->probe_hotplug_work, i915_probe_hotplug_work_func);
	INIT_WORK(&dev_priv->rps.work, gen6_pm_rps_work);
	INIT_WORK(&dev_priv->l3_parity.error_work, ivybridge_parity_work);
	INIT_DELAYED_WORK(&dev_priv->simulate_work,
			intel_hpd_simulate_reconnect_work);

	/* Let's track the enabled rps events */
	dev_priv->pm_rps_events = GEN6_PM_RPS_EVENTS;

#ifdef CONFIG_HDMI_DLP
	dev_priv->hotplug_delayed_work_enabled = 1;
	INIT_DELAYED_WORK(&dev_priv->hotplug_delayed_work, i915_hotplug_delayed_work_func);
	INIT_DELAYED_WORK(&(dual_display_status.delay_work), dual_display_delay_work);
#else
	if (HAS_PCH_CPT(dev)) {
		dev_priv->hotplug_delayed_work_enabled = 1;
		INIT_DELAYED_WORK(&dev_priv->hotplug_delayed_work, i915_hotplug_delayed_work_func);
	 } else
		dev_priv->hotplug_delayed_work_enabled = 0;
#endif

	setup_timer(&dev_priv->hotplug_reenable_timer, intel_hpd_irq_reenable,
		    (unsigned long) dev_priv);

	pm_qos_add_request(&dev_priv->pm_qos, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);

	if (IS_GEN2(dev)) {
		dev->max_vblank_count = 0;
		dev->driver->get_vblank_counter = i8xx_get_vblank_counter;
	} else if (IS_G4X(dev) || INTEL_INFO(dev)->gen >= 5) {
		dev->max_vblank_count = 0xffffffff; /* full 32 bit counter */
		dev->driver->get_vblank_counter = gm45_get_vblank_counter;
	} else {
		dev->driver->get_vblank_counter = i915_get_vblank_counter;
		dev->max_vblank_count = 0xffffff; /* only 24 bits of frame count */
	}

	if (drm_core_check_feature(dev, DRIVER_MODESET)) {
		dev->driver->get_vblank_timestamp = i915_get_vblank_timestamp;
		dev->driver->get_scanout_position = i915_get_crtc_scanoutpos;
	}

	if (IS_CHERRYVIEW(dev)) {
		dev->driver->irq_handler = cherryview_irq_handler;
		dev->driver->irq_preinstall = cherryview_irq_preinstall;
		dev->driver->irq_postinstall = cherryview_irq_postinstall;
		dev->driver->irq_uninstall = cherryview_irq_uninstall;
		dev->driver->enable_vblank = valleyview_enable_vblank;
		dev->driver->disable_vblank = valleyview_disable_vblank;
		dev_priv->display.hpd_irq_setup = i915_hpd_irq_setup;
	} else if (IS_VALLEYVIEW(dev)) {
		dev->driver->irq_handler = valleyview_irq_handler;
		dev->driver->irq_preinstall = valleyview_irq_preinstall;
		dev->driver->irq_postinstall = valleyview_irq_postinstall;
		dev->driver->irq_uninstall = valleyview_irq_uninstall;
		dev->driver->enable_vblank = valleyview_enable_vblank;
		dev->driver->disable_vblank = valleyview_disable_vblank;
		dev_priv->display.hpd_irq_setup = i915_hpd_irq_setup;
	} else if (IS_GEN8(dev)) {
		dev->driver->irq_handler = gen8_irq_handler;
		dev->driver->irq_preinstall = gen8_irq_reset;
		dev->driver->irq_postinstall = gen8_irq_postinstall;
		dev->driver->irq_uninstall = gen8_irq_uninstall;
		dev->driver->enable_vblank = gen8_enable_vblank;
		dev->driver->disable_vblank = gen8_disable_vblank;
		dev_priv->display.hpd_irq_setup = ibx_hpd_irq_setup;
	} else if (HAS_PCH_SPLIT(dev)) {
		dev->driver->irq_handler = ironlake_irq_handler;
		dev->driver->irq_preinstall = ironlake_irq_reset;
		dev->driver->irq_postinstall = ironlake_irq_postinstall;
		dev->driver->irq_uninstall = ironlake_irq_uninstall;
		dev->driver->enable_vblank = ironlake_enable_vblank;
		dev->driver->disable_vblank = ironlake_disable_vblank;
		dev_priv->display.hpd_irq_setup = ibx_hpd_irq_setup;
	} else {
		if (INTEL_INFO(dev)->gen == 2) {
			dev->driver->irq_preinstall = i8xx_irq_preinstall;
			dev->driver->irq_postinstall = i8xx_irq_postinstall;
			dev->driver->irq_handler = i8xx_irq_handler;
			dev->driver->irq_uninstall = i8xx_irq_uninstall;
		} else if (INTEL_INFO(dev)->gen == 3) {
			dev->driver->irq_preinstall = i915_irq_preinstall;
			dev->driver->irq_postinstall = i915_irq_postinstall;
			dev->driver->irq_uninstall = i915_irq_uninstall;
			dev->driver->irq_handler = i915_irq_handler;
			dev_priv->display.hpd_irq_setup = i915_hpd_irq_setup;
		} else {
			dev->driver->irq_preinstall = i965_irq_preinstall;
			dev->driver->irq_postinstall = i965_irq_postinstall;
			dev->driver->irq_uninstall = i965_irq_uninstall;
			dev->driver->irq_handler = i965_irq_handler;
			dev_priv->display.hpd_irq_setup = i915_hpd_irq_setup;
		}
		dev->driver->enable_vblank = i915_enable_vblank;
		dev->driver->disable_vblank = i915_disable_vblank;
	}
}

void intel_hpd_init(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_mode_config *mode_config = &dev->mode_config;
	struct drm_connector *connector;
	unsigned long irqflags;
	int i;

	for (i = 1; i < HPD_NUM_PINS; i++) {
		dev_priv->hpd_stats[i].hpd_cnt = 0;
		dev_priv->hpd_stats[i].hpd_mark = HPD_ENABLED;
	}
	list_for_each_entry(connector, &mode_config->connector_list, head) {
		struct intel_connector *intel_connector = to_intel_connector(connector);
		connector->polled = intel_connector->polled;
		if (!connector->polled && I915_HAS_HOTPLUG(dev) && intel_connector->encoder->hpd_pin > HPD_NONE)
			connector->polled = DRM_CONNECTOR_POLL_HPD;
	}

	/* Interrupt setup is already guaranteed to be single-threaded, this is
	 * just to make the assert_spin_locked checks happy. */
	spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
	if (dev_priv->display.hpd_irq_setup)
		dev_priv->display.hpd_irq_setup(dev);
	spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);
}

/* Disable interrupts so we can allow runtime PM. */
void intel_runtime_pm_disable_interrupts(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	dev->driver->irq_uninstall(dev);
	dev_priv->pm.irqs_disabled = true;
}

/* Restore interrupts so we can recover from runtime PM. */
void intel_runtime_pm_restore_interrupts(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long flags;

	dev_priv->pm.irqs_disabled = false;
	dev->driver->irq_preinstall(dev);
	dev->driver->irq_postinstall(dev);

	spin_lock_irqsave(&dev_priv->irq_lock, flags);
	if (dev_priv->display.hpd_irq_setup)
		dev_priv->display.hpd_irq_setup(dev);
	spin_unlock_irqrestore(&dev_priv->irq_lock, flags);
}
