// SPDX-License-Identifier: GPL-2.0
/*
 * Based on panel-truly-nt35597.c
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>

static const char * const regulator_names[] = {
  "vdda",
  "vdispp",
  "vdispn",
};

static unsigned long const regulator_enable_loads[] = {
  62000,
  100000,
  100000,
};

static unsigned long const regulator_disable_loads[] = {
  80,
  100,
  100,
};

struct cmd_set {
  u8 commands[27];
  u8 size;
};

struct lg494x_config {
  u32 width_mm;
  u32 height_mm;
  const char *panel_name;
  const struct cmd_set *panel_on_cmds;
  u32 num_on_cmds;
  const struct drm_display_mode *dm;
};

struct lge_lg494x {
  struct device *dev;
  struct drm_panel panel;

  struct regulator_bulk_data supplies[ARRAY_SIZE(regulator_names)];

  struct gpio_desc *reset_gpio;
  struct gpio_desc *mode_gpio;

  struct backlight_device *backlight;

  struct mipi_dsi_device *dsi[2];

  const struct lg494x_config *config;
  bool prepared;
  bool enabled;
};

static inline struct lge_lg494x *panel_to_ctx(struct drm_panel *panel)
{
  return container_of(panel, struct lge_lg494x, panel);
}

static const struct cmd_set qcom_2k_panel_on_magic_cmds[] = {
/** Page Address Set */
/*39 01 00 00 00 00 05*/ {{0x2B, 0x00, 0x00, 0x09, 0xFF}, 5},
/** Manufacturer Command Protection */
/*15 01 00 00 00 00 02*/ {{0xB0, 0xAC}, 2},
/** Mailbox for Touch Register access */
/*39 01 00 00 00 00 05*/ {{0xCC, 0x01, 0x00, 0x0B, 0x04}, 5},
/*39 01 00 00 00 00 05*/ {{0xCC, 0x00, 0x00, 0x0B, 0x04}, 5},
/** Partial Area */
/*39 01 00 00 00 00 05*/ {{0x30, 0x00, 0x00, 0x02, 0xAB}, 5},
/** Tearing Effect Line on */
/*15 01 00 00 00 00 02*/ {{0x35, 0x00}, 2},
/** Write Control Display */
/*15 01 00 00 00 00 02*/ {{0x51, 0xFF}, 2},
/** Write Control Display */
/*15 01 00 00 00 00 02*/ {{0x53, 0x2D}, 2},
/** Write Content Adaptive Brightness Control */
/*15 01 00 00 00 00 02*/ {{0x55, 0x80}, 2},
/** Display Control 1 */
/*39 01 00 00 00 00 08*/ {{0xB3, 0x0A, 0x14, 0x28, 0xC8, 0x1A, 0x94, 0x02}, 8},
/** Display Control 2 */
/*39 01 00 00 00 00 0B*/ {{0xB4, 0x91, 0x08, 0x0A, 0x0A, 0x0A, 0x0A, 0x14, 0x14, 0x14, 0x14}, 11},
/** Display Control 3 */
/*39 01 00 00 00 00 0F*/ {{0xB5, 0x28, 0x10, 0x20, 0xC8, 0x22, 0x09, 0x09, 0x01, 0x01, 0x50, 0x68, 0xE8, 0x05, 0x05}, 15},
/** Display Control 4 */
/*39 01 00 00 00 00 05*/ {{0xB6, 0x16, 0x0E, 0x0C, 0x00}, 5},
/** Panel Setting */
/*39 01 00 00 00 00 07*/ {{0xB8, 0x51, 0x91, 0x7F, 0x00, 0x70, 0x2A}, 7},
/** Internal Oscillator Setting */
/*39 01 00 00 00 00 03*/ {{0xC0, 0x91, 0x00}, 3},
/** Power Control 1 */
/*39 01 00 00 00 00 07*/ {{0xC1, 0x01, 0xE0, 0xF0, 0xC2, 0xCF, 0xF0}, 7},
/** Power Control 2 */
/*15 01 00 00 00 00 02*/ {{0xC2, 0xCC}, 2},
/** Power Control 3 */
/*39 01 00 00 00 00 08*/ {{0xC3, 0x35, 0x44, 0x22, 0x26, 0x21, 0x55, 0xDD}, 8},
/** Power Control 4 */
/*39 01 00 00 00 00 04*/ {{0xC4, 0xA2, 0xA4, 0xA4}, 4},
/** Abrupt Power Off Control */
/*39 01 00 00 00 00 03*/ {{0xC9, 0x9F, 0x00}, 3},
/** U2 Control */
/*39 01 00 00 00 00 0C*/ {{0xCB, 0x86, 0x86, 0x77, 0x06, 0x01, 0x08, 0x16, 0x0E, 0x0C, 0x00, 0x36}, 12},
/** Touch Control */
/*39 01 00 00 00 00 1A*/ {{0xCD, 0xF8, 0x48, 0x08, 0x3C, 0x33, 0x23, 0x22, 0x06, 0x04, 0x01, 0x03, 0x57, 0xAC, 0x04, 0x04, 0xBC, 0x44, 0x08, 0x50, 0x0A, 0x34, 0xAA, 0x00, 0x00, 0x02}, 26},
/** Positive Gamma Curve for Red */
/*39 01 00 00 00 00 0E*/ {{0xD0, 0x00, 0x12, 0x20, 0x2E, 0x39, 0x40, 0x4E, 0x46, 0x39, 0x2A, 0x14, 0x00, 0x03}, 14},
/** Negative Gamma Curve for Red */
/*39 01 00 00 00 00 0E*/ {{0xD1, 0x00, 0x12, 0x20, 0x2E, 0x39, 0x40, 0x4E, 0x46, 0x39, 0x2A, 0x14, 0x04, 0x03}, 14},
/** Positive Gamma Curve for  Green */
/*39 01 00 00 00 00 0E*/ {{0xD2, 0x00, 0x12, 0x1F, 0x2D, 0x38, 0x3F, 0x4F, 0x47, 0x3A, 0x2A, 0x14, 0x00, 0x03}, 14},
/** Negative Gamma Curve for Green */
/*39 01 00 00 00 00 0E*/ {{0xD3, 0x00, 0x12, 0x1F, 0x2D, 0x38, 0x3F, 0x4F, 0x47, 0x3A, 0x2A, 0x14, 0x04, 0x03}, 14},
/** Positive Gamma Curve for Blue */
/*39 01 00 00 00 00 0E*/ {{0xD4, 0x00, 0x12, 0x20, 0x2E, 0x39, 0x40, 0x4E, 0x46, 0x39, 0x2A, 0x14, 0x00, 0x03}, 14},
/** Negative Gamma Curve for Blue */
/*39 01 00 00 00 00 0E*/ {{0xD5, 0x00, 0x12, 0x20, 0x2E, 0x39, 0x40, 0x4E, 0x46, 0x39, 0x2A, 0x14, 0x04, 0x03}, 14},
/** Left Side GIP Pad Setting */
/*39 01 00 00 00 00 16*/ {{0xD6, 0x22, 0x6D, 0x6C, 0x6E, 0x22, 0x61, 0x22, 0x65, 0x67, 0x69, 0x6B, 0x22, 0x71, 0x59, 0x95, 0x6A, 0x03, 0x59, 0x95, 0x6A, 0x03}, 22},
/** Right Side GIP Pad Setting */
/*39 01 00 00 00 00 16*/ {{0xD7, 0x22, 0x6D, 0x6C, 0x6E, 0x22, 0x60, 0x22, 0x64, 0x66, 0x68, 0x6A, 0x22, 0x71, 0x59, 0x95, 0x6A, 0x03, 0x59, 0x95, 0x6A, 0x03}, 22},
/** Left Side MUX Pad Setting */
/*39 01 00 00 00 00 07*/ {{0xD8, 0x78, 0x77, 0x76, 0x75, 0x74, 0x73}, 7},
/** Right Side MUX Pad Setting */
/*39 01 00 00 00 00 07*/ {{0xD9, 0x78, 0x77, 0x76, 0x75, 0x74, 0x73}, 7},
/** LFD Control */
/*39 01 00 00 00 00 08*/ {{0xCA, 0xBD, 0xBD, 0xCC, 0x0C, 0x00, 0x93, 0x00}, 8},
/** OTP1 */
/*39 01 00 00 00 00 04*/ {{0xE8, 0x00, 0x00, 0x00}, 4},
/** Image Enhancement Control 1 */
/*15 01 00 00 00 00 02*/ {{0xF0, 0x42}, 2},
/** Image Enhancement Control SH */
/*39 01 00 00 00 00 05*/ {{0xF2, 0x01, 0x00, 0x04, 0x00}, 5},
/** Image Enhancement Control CSC */
/*39 01 00 00 00 00 18*/ {{0xF3, 0x00, 0x46, 0x86, 0xC0, 0xFF, 0x30, 0x00, 0x40, 0x80, 0xC0, 0xFF, 0x50, 0x00, 0x40, 0x80, 0xC0, 0xFF, 0x00, 0x00, 0x40, 0x80, 0xC0, 0xFF}, 24},
/** Backlight Control */
/*39 01 00 00 00 00 07*/ {{0xB9, 0x13, 0x00, 0x00, 0x78, 0x7F, 0x80}, 7},
/** 1st CABC Parametrs for NEW CABC algorithm */
/*39 01 00 00 00 00 18*/ {{0xFE, 0x20, 0x40, 0x60, 0x80, 0xA0, 0xC0, 0xE0, 0x10, 0x13, 0x14, 0x03, 0x18, 0x17, 0x14, 0x00, 0xCD, 0xD3, 0xDA, 0xE0, 0xE8, 0xF5, 0xFA, 0xFF}, 24},
/** 2nd CABC Parametrs for NEW CABC algorithm */
/*39 01 00 00 00 00 0D*/ {{0xFF, 0x88, 0x11, 0x22, 0x02, 0x52, 0xC2, 0x02, 0x00, 0x0A, 0x00, 0x02, 0xD0}, 13},

/** SRE Control */
/** Disable the SRE by default */
/*39 01 00 00 00 00 19*/ {{0xFC, 0x0F, 0x60, 0x75, 0x87, 0xA8, 0xC0, 0xE4, 0xF0, 0x00, 0x40, 0x70, 0x90, 0xA0, 0xB0, 0xC0, 0xF0, 0x00, 0x20, 0x40, 0x60, 0x80, 0xA0, 0xC0, 0xE0}, 25},

/** RLDT report off */
/*39 01 00 00 00 00 05*/ {{0xEB, 0x38, 0xC1, 0x00, 0x00}, 5},

/** Display On */
/*05 01 00 00 00 00 01*/ {{0x29}, 1},
/** Sleep Out + 50ms */
/*05 01 00 00 32 00 01*/ {{0x11}, 1}
};

static int lge_dcs_write(struct drm_panel *panel, u32 command)
{
  struct lge_lg494x *ctx = panel_to_ctx(panel);
  int i, ret;

  for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++) {
    ret = mipi_dsi_dcs_write(ctx->dsi[i], command, NULL, 0);
    if (ret < 0) {
      DRM_DEV_ERROR(ctx->dev,
        "cmd 0x%x failed for dsi = %d\n",
        command, i);
    }
  }

  return ret;
}

static int lge_dcs_write_buf(struct drm_panel *panel,
  u32 size, const u8 *buf)
{
  struct lge_lg494x *ctx = panel_to_ctx(panel);
  int ret = 0;
  int i;

  for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++) {
    ret = mipi_dsi_dcs_write_buffer(ctx->dsi[i], buf, size);
    if (ret < 0) {
      DRM_DEV_ERROR(ctx->dev,
        "failed to tx cmd [%d], err: %d\n", i, ret);
      return ret;
    }
  }

  return ret;
}

static int lge_lg494x_power_on(struct lge_lg494x *ctx)
{
  return 0;
/*  int ret, i;

  for (i = 0; i < ARRAY_SIZE(ctx->supplies); i++) {
    ret = regulator_set_load(ctx->supplies[i].consumer,
          regulator_enable_loads[i]);
    if (ret)
      return ret;
  }

  ret = regulator_bulk_enable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
  if (ret < 0)
    return ret;*/

  /*
   * Reset sequence of lge panel requires the panel to be
   * out of reset for 10ms, followed by being held in reset
   * for 10ms and then out again
   */
  gpiod_set_value(ctx->reset_gpio, 1);
  usleep_range(10000, 20000);
  gpiod_set_value(ctx->reset_gpio, 0);
  usleep_range(10000, 20000);
  gpiod_set_value(ctx->reset_gpio, 1);
  usleep_range(10000, 20000);

  return 0;
}

static int lge_lg494x_power_off(struct lge_lg494x *ctx)
{
  return 0;
  int ret = 0;
  int i;

  gpiod_set_value(ctx->reset_gpio, 1);

/*  for (i = 0; i < ARRAY_SIZE(ctx->supplies); i++) {
    ret = regulator_set_load(ctx->supplies[i].consumer,
        regulator_disable_loads[i]);
    if (ret) {
      DRM_DEV_ERROR(ctx->dev,
        "regulator_set_load failed %d\n", ret);
      return ret;
    }
  }

  ret = regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
  if (ret) {
    DRM_DEV_ERROR(ctx->dev,
      "regulator_bulk_disable failed %d\n", ret);
  }*/
  return ret;
}

static int lge_lg494x_disable(struct drm_panel *panel)
{
  struct lge_lg494x *ctx = panel_to_ctx(panel);
  int ret;

  if (!ctx->enabled)
    return 0;

  if (ctx->backlight) {
    ret = backlight_disable(ctx->backlight);
    if (ret < 0)
      DRM_DEV_ERROR(ctx->dev, "backlight disable failed %d\n",
        ret);
  }

  ctx->enabled = false;
  return 0;
}

static int lge_lg494x_unprepare(struct drm_panel *panel)
{
  struct lge_lg494x *ctx = panel_to_ctx(panel);
  int ret = 0;

  if (!ctx->prepared)
    return 0;

  ctx->dsi[0]->mode_flags = 0;
  ctx->dsi[1]->mode_flags = 0;

  ret = lge_dcs_write(panel, MIPI_DCS_SET_DISPLAY_OFF);
  if (ret < 0) {
    DRM_DEV_ERROR(ctx->dev,
      "set_display_off cmd failed ret = %d\n",
      ret);
  }

  /* 120ms delay required here as per DCS spec */
  msleep(120);

  ret = lge_dcs_write(panel, MIPI_DCS_ENTER_SLEEP_MODE);
  if (ret < 0) {
    DRM_DEV_ERROR(ctx->dev,
      "enter_sleep cmd failed ret = %d\n", ret);
  }

  ret = lge_lg494x_power_off(ctx);
  if (ret < 0)
    DRM_DEV_ERROR(ctx->dev, "power_off failed ret = %d\n", ret);

  ctx->prepared = false;
  return ret;
}

static int lge_lg494x_prepare(struct drm_panel *panel)
{
  struct lge_lg494x *ctx = panel_to_ctx(panel);
  int ret;
  int i;
  const struct cmd_set *panel_on_cmds;
  const struct lg494x_config *config;
  u32 num_cmds;

  if (ctx->prepared)
    return 0;

  ret = lge_lg494x_power_on(ctx);
  if (ret < 0)
    return ret;

  ctx->dsi[0]->mode_flags |= MIPI_DSI_MODE_LPM;
  ctx->dsi[1]->mode_flags |= MIPI_DSI_MODE_LPM;

  config = ctx->config;
  panel_on_cmds = config->panel_on_cmds;
  num_cmds = config->num_on_cmds;

  for (i = 0; i < num_cmds; i++) {
    ret = lge_dcs_write_buf(panel,
        panel_on_cmds[i].size,
          panel_on_cmds[i].commands);
    if (ret < 0) {
      DRM_DEV_ERROR(ctx->dev,
        "cmd set tx failed i = %d ret = %d\n",
          i, ret);
      goto power_off;
    }
  }

  ret = lge_dcs_write(panel, MIPI_DCS_EXIT_SLEEP_MODE);
  if (ret < 0) {
    DRM_DEV_ERROR(ctx->dev,
      "exit_sleep_mode cmd failed ret = %d\n",
      ret);
    goto power_off;
  }

  /* Per DSI spec wait 120ms after sending exit sleep DCS command */
  msleep(120);

  ret = lge_dcs_write(panel, MIPI_DCS_SET_DISPLAY_ON);
  if (ret < 0) {
    DRM_DEV_ERROR(ctx->dev,
      "set_display_on cmd failed ret = %d\n", ret);
    goto power_off;
  }

  /* Per DSI spec wait 120ms after sending set_display_on DCS command */
  msleep(120);

  ctx->prepared = true;

  return 0;

power_off:
  if (lge_lg494x_power_off(ctx))
    DRM_DEV_ERROR(ctx->dev, "power_off failed\n");
  return ret;
}

static int lge_lg494x_enable(struct drm_panel *panel)
{
  struct lge_lg494x *ctx = panel_to_ctx(panel);
  int ret;

  if (ctx->enabled)
    return 0;

  if (ctx->backlight) {
    ret = backlight_enable(ctx->backlight);
    if (ret < 0)
      DRM_DEV_ERROR(ctx->dev, "backlight enable failed %d\n",
              ret);
  }

  ctx->enabled = true;

  return 0;
}

static int lge_lg494x_get_modes(struct drm_panel *panel,
           struct drm_connector *connector)
{
  struct lge_lg494x *ctx = panel_to_ctx(panel);
  struct drm_display_mode *mode;
  const struct lg494x_config *config;

  config = ctx->config;
  mode = drm_mode_create(connector->dev);
  if (!mode) {
    DRM_DEV_ERROR(ctx->dev,
      "failed to create a new display mode\n");
    return 0;
  }

  connector->display_info.width_mm = config->width_mm;
  connector->display_info.height_mm = config->height_mm;
  drm_mode_copy(mode, config->dm);
  mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
  drm_mode_probed_add(connector, mode);

  return 1;
}

static const struct drm_panel_funcs lge_lg494x_drm_funcs = {
  .disable = lge_lg494x_disable,
  .unprepare = lge_lg494x_unprepare,
  .prepare = lge_lg494x_prepare,
  .enable = lge_lg494x_enable,
  .get_modes = lge_lg494x_get_modes,
};

static int lge_lg494x_panel_add(struct lge_lg494x *ctx)
{
  struct device *dev = ctx->dev;
  int ret, i;

  for (i = 0; i < ARRAY_SIZE(ctx->supplies); i++)
    ctx->supplies[i].supply = regulator_names[i];

  ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->supplies),
              ctx->supplies);
  if (ret < 0)
    return ret;

  ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
  if (IS_ERR(ctx->reset_gpio)) {
    DRM_DEV_ERROR(dev, "cannot get reset gpio %ld\n",
      PTR_ERR(ctx->reset_gpio));
    return PTR_ERR(ctx->reset_gpio);
  }

  // ctx->mode_gpio = devm_gpiod_get(dev, "mode", GPIOD_OUT_LOW);
  // if (IS_ERR(ctx->mode_gpio)) {
  //   DRM_DEV_ERROR(dev, "cannot get mode gpio %ld\n",
  //     PTR_ERR(ctx->mode_gpio));
  //   return PTR_ERR(ctx->mode_gpio);
  // }

  // /* dual port */
  // gpiod_set_value(ctx->mode_gpio, 0);

  drm_panel_init(&ctx->panel, dev, &lge_lg494x_drm_funcs,
           DRM_MODE_CONNECTOR_DSI);
  drm_panel_add(&ctx->panel);

  return 0;
}

static const struct drm_display_mode lg_alice_2k_mode = {
  .name = "1440x2560",
  .clock = 150000 * 2,
  .hdisplay = (720) * 2,
  .hsync_start = (720 + 60) * 2,
  .hsync_end = (720 + 60 + 12) * 2,
  .htotal = (720 + 60 + 12 + 80) * 2,
  .vdisplay = 2560,
  .vsync_start = 2560 + 150,
  .vsync_end = 2560 + 150 + 1,
  .vtotal = 2560 + 150 + 1 + 155,
  .flags = 0,
};

static const struct lg494x_config lg494x_dir = {
  .width_mm = 66,
  .height_mm = 117,
  .panel_name = "lg_alice_2k_panel",
  .dm = &lg_alice_2k_mode,
  .panel_on_cmds = qcom_2k_panel_on_magic_cmds,
  .num_on_cmds = ARRAY_SIZE(qcom_2k_panel_on_magic_cmds),
};

static int lge_lg494x_probe(struct mipi_dsi_device *dsi)
{
  struct device *dev = &dsi->dev;
  struct lge_lg494x *ctx;
  struct mipi_dsi_device *dsi1_device;
  struct device_node *dsi1;
  struct mipi_dsi_host *dsi1_host;
  struct mipi_dsi_device *dsi_dev;
  int ret = 0;
  int i;

  const struct mipi_dsi_device_info info = {
    .type = "lg494x",
    .channel = 0,
    .node = NULL,
  };

  ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);

  if (!ctx)
    return -ENOMEM;

  /*
   * This device represents itself as one with two input ports which are
   * fed by the output ports of the two DSI controllers . The DSI0 is
   * the master controller and has most of the panel related info in its
   * child node.
   */

  ctx->config = of_device_get_match_data(dev);

  if (!ctx->config) {
    dev_err(dev, "missing device configuration\n");
    return -ENODEV;
  }

  dsi1 = of_graph_get_remote_node(dsi->dev.of_node, 1, -1);
  if (!dsi1) {
    DRM_DEV_ERROR(dev,
      "failed to get remote node for dsi1_device\n");
    return -ENODEV;
  }

  dsi1_host = of_find_mipi_dsi_host_by_node(dsi1);
  of_node_put(dsi1);
  if (!dsi1_host) {
    DRM_DEV_ERROR(dev, "failed to find dsi host\n");
    return -EPROBE_DEFER;
  }

  /* register the second DSI device */
  dsi1_device = mipi_dsi_device_register_full(dsi1_host, &info);
  if (IS_ERR(dsi1_device)) {
    DRM_DEV_ERROR(dev, "failed to create dsi device\n");
    return PTR_ERR(dsi1_device);
  }

  mipi_dsi_set_drvdata(dsi, ctx);

  ctx->dev = dev;
  ctx->dsi[0] = dsi;
  ctx->dsi[1] = dsi1_device;

  ret = lge_lg494x_panel_add(ctx);
  if (ret) {
    DRM_DEV_ERROR(dev, "failed to add panel\n");
    goto err_panel_add;
  }

  for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++) {
    dsi_dev = ctx->dsi[i];
    dsi_dev->lanes = 4;
    dsi_dev->format = MIPI_DSI_FMT_RGB888;
    dsi_dev->mode_flags = 0;
    ret = mipi_dsi_attach(dsi_dev);
    if (ret < 0) {
      DRM_DEV_ERROR(dev,
        "dsi attach failed i = %d\n", i);
      goto err_dsi_attach;
    }
  }

  return 0;

err_dsi_attach:
  drm_panel_remove(&ctx->panel);
err_panel_add:
  mipi_dsi_device_unregister(dsi1_device);
  return ret;
}

static int lge_lg494x_remove(struct mipi_dsi_device *dsi)
{
  struct lge_lg494x *ctx = mipi_dsi_get_drvdata(dsi);

  if (ctx->dsi[0])
    mipi_dsi_detach(ctx->dsi[0]);
  if (ctx->dsi[1]) {
    mipi_dsi_detach(ctx->dsi[1]);
    mipi_dsi_device_unregister(ctx->dsi[1]);
  }

  drm_panel_remove(&ctx->panel);
  return 0;
}

static const struct of_device_id lge_lg494x_of_match[] = {
  {
    .compatible = "lge,lg4945-2k-display",
    .data = &lg494x_dir,
  },
  {
    .compatible = "lge,lg4946-2k-display",
    .data = &lg494x_dir,
  },
  { }
};
MODULE_DEVICE_TABLE(of, lge_lg494x_of_match);

static struct mipi_dsi_driver lge_lg494x_driver = {
  .driver = {
    .name = "panel-lge-lg494x",
    .of_match_table = lge_lg494x_of_match,
  },
  .probe = lge_lg494x_probe,
  .remove = lge_lg494x_remove,
};
module_mipi_dsi_driver(lge_lg494x_driver);

MODULE_DESCRIPTION("LG Electronics lg4945/lg4945 DSI Panel Driver");
MODULE_LICENSE("GPL v2");
