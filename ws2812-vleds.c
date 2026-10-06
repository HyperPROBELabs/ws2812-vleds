// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2025 Sakura Pi Org <kernel@sakurapi.org>
// Copyright (c) 2026 Kali Assistant <work.kaliassistant.github@gmail.com>
// Copyright (c) 2026 HyperPROBE Labs. <hyperprobelabs@proton.me>

/*
* WS2812 virtual LED driver
*
* This driver uses SPI to drive WS2812 LEDs.
* Besides, it registers a standard LED control interface for controlling,
* dts-based RGB24 color preset and hsb brightness support.
*
* It uses 8 bits to simulate one WS2812 color bit, so this driver
* needs a 6.4MHz SPI to work properly.
*
* In some cases, you may calibrate the SPI speed manually.
*
*
* --- Fixed for 5.10.x (gnu89/C90-strict) kernel build ---
*  - moved ws2812_framebuf_t/ws2812_color_t typedefs above their first use
*    (this is what was silently turning `ws_opctx` into an `int *` and
*    causing all the "incompatible pointer type" errors)
*  - removed nested kernel min()/max() usage in rgb_to_hsl() (typeof()
*    strict-typecheck trips -Werror on uint8_t args)
*  - hoisted all "declaration after statement" locals to the top of
*    their blocks (this tree is built C90-strict)
*  - replaced size_t-in-for-loop-init with a pre-declared index
*  - devm_mutex_init() does not exist on 5.10.x -> plain mutex_init()
*  - spi_driver.remove returns int on 5.10.x, not void
*  - _color_value is now initialized to NULL before the optional
*    of_property_read_string(), which previously left it uninitialized
*    on the "property missing" path and fed a garbage pointer into
*    hexclr_to_rgb888()
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/mutex.h>

#include <linux/types.h>

#include <linux/device.h>

#include <linux/string.h>


/* ---- typedefs first: these must be visible before anything uses them ---- */

typedef struct {
  uint8_t g[8];
  uint8_t r[8];
  uint8_t b[8];
} ws2812_color_t;

typedef void* ws2812_ctx_t;

typedef enum {

  // TH + TL = 1.25us + ±600ns
  // 0 code: high voltage 0.35us, low 0.8us
  // 1 code: high voltage 0.7us,  low 0.6us

  // ws2812_bit_low
  //   _
  //  | |______
  //  1100 0000

  // ws2812_bit_high
  //   _____
  //  |     |__
  //  1111 1000

  // ws2812_bit_zero
  // for reset signal use
  //
  //  _________
  //  0000 0000

  // ws2812_bit_cali
  // this option causes spi output a 50% width clk
  // to help with the manually spi clk calibration
  //  ___
  // |   |____
  // 1111 0000

  ws2812_bit_low  = 0b11000000,
  ws2812_bit_high = 0b11111000,
  ws2812_bit_zero = 0b00000000,
  ws2812_bit_cali = 0b11110000

} ws2812_bit_t;

typedef struct {

  uint8_t* buffer;

  struct {
    ws2812_color_t* reset;
    ws2812_color_t* pixels;
    ws2812_color_t* reset2;

  } anchor;
  int pixel_count;

} ws2812_framebuf_t;


struct driver_data {
  int num_leds;

  struct {
    void* ptr;
    int length;
  } tx_buffer;

  ws2812_framebuf_t* ws_opctx;

  struct spi_device *spi;
  struct mutex mutex;

  struct list_head leds;
};

struct color24 {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

struct wsled_data {
  struct list_head list;
  struct led_classdev* cls;          // main LED device (for brightness control)

#ifdef CONFIG_WS2812_VLEDS_CHANNEL_CONTROL
  struct led_classdev* cls_red;      // red channel LED device
  struct led_classdev* cls_green;    // green channel LED device
  struct led_classdev* cls_blue;     // blue channel LED device
#endif

  struct color24 color;              // calculated color
  struct color24 origin_color;       // origin color is readonly
  uint8_t lightness; // for HSL color space
};

enum filter_type {
  filter_main = 0,

#ifdef CONFIG_WS2812_VLEDS_CHANNEL_CONTROL
  filter_red_ch,
  filter_green_ch,
  filter_blue_ch,
#endif

};


#define __make_clrbit(_ch, _bit) \
  ((_ch & _bit) ? ws2812_bit_high : ws2812_bit_low)

#define __inflate_ch(_ch) { \
    __make_clrbit(_ch, 0b10000000), \
    __make_clrbit(_ch, 0b01000000), \
    __make_clrbit(_ch, 0b00100000), \
    __make_clrbit(_ch, 0b00010000), \
    __make_clrbit(_ch, 0b00001000), \
    __make_clrbit(_ch, 0b00000100), \
    __make_clrbit(_ch, 0b00000010), \
    __make_clrbit(_ch, 0b00000001) \
  }

#define ws2812_calc_bufsize(leds) (sizeof(ws2812_color_t) * (8 + leds))


/* ---- forward declarations (definition order below still matters for a
 *      couple of these, but this keeps callers happy regardless) ---- */
static int ws2812_vleds_update(struct driver_data* drv);
static void ws2812_clear(ws2812_framebuf_t* frame, ws2812_color_t color);
static void ws2812_set_pixel(ws2812_framebuf_t* frame, int index, ws2812_color_t color);


static ws2812_color_t ws2812_rgb(uint8_t r, uint8_t g, uint8_t b) {
  return ((ws2812_color_t) {
    .r = __inflate_ch(r),
    .g = __inflate_ch(g),
    .b = __inflate_ch(b),
  });
}


static uint8_t __hex2int8(const char* c)
{
  uint8_t _value = 0;
  char _wrap[3] = {c[0], c[1], '\0'};
  if(kstrtou8(_wrap, 16, &_value) != 0) {
    return 0;
  }
  return _value;
}

static bool hexclr_validate(const char* hex_color)
{
  if(!hex_color || (*hex_color != '#'))
    return false;

  if(strlen(hex_color) != 7)
    return false;

  return true;
}

static bool hexclr_to_rgb888(const char* hex_color, uint8_t* r, uint8_t* g, uint8_t* b)
{
  if(!hexclr_validate(hex_color))
    return false;

  // accept color format like #AABBCC
  if(r) *r = __hex2int8(hex_color+1);
  if(g) *g = __hex2int8(hex_color+3);
  if(b) *b = __hex2int8(hex_color+5);

  return true;
}

static uint8_t hsl_to_rgb_component(int p, int q, int t)
{
    if (t < 0) t += 255;
    if (t > 255) t -= 255;
    if (t < 42) return p + ((q - p) * 6 * t) / 255;
    if (t < 128) return q;
    if (t < 170) return p + ((q - p) * (170 - t) * 6) / 255;
    return p;
}

static void rgb_to_hsl(uint8_t r, uint8_t g, uint8_t b, int *h, int *s, int *l)
{
    /* plain int comparisons instead of kernel min()/max(): those macros do a
     * strict typeof()-based typecheck and nesting them over uint8_t args
     * trips -Werror here */
    int r_i = r, g_i = g, b_i = b;
    int max_val, min_val, delta;

    max_val = (r_i > g_i) ? ((r_i > b_i) ? r_i : b_i) : ((g_i > b_i) ? g_i : b_i);
    min_val = (r_i < g_i) ? ((r_i < b_i) ? r_i : b_i) : ((g_i < b_i) ? g_i : b_i);
    delta = max_val - min_val;

    // Lightness (0-255)
    *l = (max_val + min_val) / 2;

    if (delta == 0) {
        *h = 0;
        *s = 0;
        return;
    }

    // Saturation (0-255)
    if (*l < 128)
        *s = (delta * 255) / (max_val + min_val);
    else
        *s = (delta * 255) / (510 - max_val - min_val);

    // Hue (0-359)
    if (max_val == r_i)
        *h = ((g_i - b_i) * 60) / delta;
    else if (max_val == g_i)
        *h = 120 + ((b_i - r_i) * 60) / delta;
    else
        *h = 240 + ((r_i - g_i) * 60) / delta;

    if (*h < 0) *h += 360;
}

static void hsl_to_rgb(int h, int s, int l, uint8_t *r, uint8_t *g, uint8_t *b)
{
    int q, p, h_norm;

    if (s == 0) {
        *r = *g = *b = l;
        return;
    }

    q = (l < 128) ? (l * (255 + s)) / 255 : l + s - (l * s) / 255;
    p = 2 * l - q;

    h_norm = (h * 255) / 360;

    *r = hsl_to_rgb_component(p, q, h_norm + 85);
    *g = hsl_to_rgb_component(p, q, h_norm);
    *b = hsl_to_rgb_component(p, q, h_norm - 85);
}


static int ws2812_init(struct device* dev, int leds, void* buffer, ws2812_framebuf_t** frame) {

  ws2812_framebuf_t* _alloc;

  if(!buffer) return -ENOMEM;

  _alloc = (ws2812_framebuf_t *)
    devm_kzalloc(dev, sizeof(ws2812_framebuf_t), GFP_KERNEL);
  if (!_alloc) return -ENOMEM;

  // prepare led buffer
  *frame = _alloc;
  _alloc->pixel_count = leds;
  _alloc->buffer = buffer;
  _alloc->anchor.reset = (ws2812_color_t *)_alloc->buffer;
  _alloc->anchor.pixels = _alloc->anchor.reset + 4;
  _alloc->anchor.reset2 = _alloc->anchor.pixels + leds;

  // the reset signals
  memset(_alloc->anchor.reset, ws2812_bit_zero, sizeof(ws2812_color_t) * 4);
  memset(_alloc->anchor.reset2, ws2812_bit_zero, sizeof(ws2812_color_t) * 4);

  // clear with black(all off)
  ws2812_clear(_alloc, ws2812_rgb(0, 0, 0));

  return 0;
}

static void ws2812_clear(ws2812_framebuf_t* frame, ws2812_color_t color) {
  size_t i;
  for(i = 0; i < frame->pixel_count; ++i) {
    frame->anchor.pixels[i] = color;
  }
}

static void ws2812_set_pixel(ws2812_framebuf_t* frame, int index, ws2812_color_t color) {
  frame->anchor.pixels[index] = color;
}

static void __set_lightness_color24(struct color24* src,
struct color24* dst, uint8_t lightness)
{
  int h, s, l;

  if (!src || !dst) return;

  // convert rgb to hsl
  rgb_to_hsl(src->r, src->g, src->b, &h, &s, &l);

  // agjust the lightness
  l = (l * lightness / 255);

  // convert back to rgb
  hsl_to_rgb(h, s, l, &dst->r, &dst->g, &dst->b);
}

static int __compare_set_brightness(struct led_classdev* led,
enum led_brightness bright, enum filter_type filter)
{
  struct driver_data* _drv_data = NULL;
  int _index = 0, _ret = 0;
  struct wsled_data* _node = NULL;

  _drv_data = (struct driver_data*)dev_get_drvdata(led->dev->parent);
  if(!_drv_data) {
    pr_info("failed to get drv context\n");
    return -ENODEV;
  }

  #define search_set_wsled_channel(cls, _do) \
    _index = 0; \
    list_for_each_entry(_node, &_drv_data->leds, list) { \
      if(_node->cls == led) _do \
      ++_index; \
    }

  switch(filter) {
#ifdef CONFIG_WS2812_VLEDS_CHANNEL_CONTROL
    case filter_red_ch:
      search_set_wsled_channel(cls_red, {
        _node->color.r = bright;
        break;
      });
      break;
    case filter_green_ch:
      search_set_wsled_channel(cls_green, {
        _node->color.g = bright;
        break;
      });
      break;
    case filter_blue_ch:
      search_set_wsled_channel(cls_blue, {
        _node->color.b = bright;
        break;
      });
      break;
#endif
    case filter_main:
      list_for_each_entry(_node, &_drv_data->leds, list) {
        if(_node->cls == led) {
          _node->lightness = bright;
          __set_lightness_color24(&_node->origin_color, &_node->color, _node->lightness);
          break;
        }
      }
      break;
  }

  // update leds
  ws2812_set_pixel(_drv_data->ws_opctx, _index,
    ws2812_rgb(_node->color.r, _node->color.g, _node->color.b));
  _ret = ws2812_vleds_update(_drv_data);

  return _ret;
}

#ifdef CONFIG_WS2812_VLEDS_CHANNEL_CONTROL
static int __cb_set_wsled_red(struct led_classdev* led, enum led_brightness bright) { return __compare_set_brightness(led, bright, filter_red_ch); }
static int __cb_set_wsled_green(struct led_classdev* led, enum led_brightness bright) { return __compare_set_brightness(led, bright, filter_green_ch); }
static int __cb_set_wsled_blue(struct led_classdev* led, enum led_brightness bright) { return __compare_set_brightness(led, bright, filter_blue_ch); }
#endif
static int __cb_set_wsled(struct led_classdev* led, enum led_brightness bright) { return __compare_set_brightness(led, bright, filter_main); }

static int ws2812_vleds_get_lednum(struct device_node* node) {
  struct device_node* _entry;
  int _counter = 0;
  struct device_node *child;

  _entry = of_get_child_by_name(node, "leds");
  if(!_entry) return 0;

  for_each_child_of_node(_entry, child) ++_counter;

  return _counter;
}

static int ws2812_vleds_update(struct driver_data* drv) {

  int _ret = 0;
  mutex_lock(&drv->mutex);
  _ret = spi_write(drv->spi, drv->tx_buffer.ptr, drv->tx_buffer.length);
  mutex_unlock(&drv->mutex);

  return _ret;
}

static int ws2812_vleds_probe(struct spi_device *spi)
{
  int ret;
  int _leds;
  struct driver_data* _drv_data;
  struct device_node *_enrty, *child;

  _leds = ws2812_vleds_get_lednum(spi->dev.of_node);
  if(_leds == 0) {
    dev_info(&spi->dev, "ws2812 found no leds under the controller, return.\n");
    return -ENODEV;
  }

  // fill driver ctx
  _drv_data = devm_kzalloc(&spi->dev, sizeof(*_drv_data), GFP_KERNEL);
  if (!_drv_data) return -ENOMEM;

  // save spi dev
  _drv_data->spi = spi;

  // init list
  INIT_LIST_HEAD(&_drv_data->leds);

  // create mutex (5.10.x has no devm_mutex_init(), use plain mutex_init())
  mutex_init(&_drv_data->mutex);

  // allocate tx buffer
  _drv_data->num_leds = _leds;
  _drv_data->tx_buffer.length = ws2812_calc_bufsize(_leds);
  _drv_data->tx_buffer.ptr = devm_kzalloc(&spi->dev, _drv_data->tx_buffer.length, GFP_KERNEL);
  dev_info(&spi->dev, "ws2812 txbuf allocated: %p\n", _drv_data->tx_buffer.ptr);
  if (!_drv_data->tx_buffer.ptr) return -ENOMEM;

  // wrap the buffer
  ret = ws2812_init(&spi->dev, _leds, _drv_data->tx_buffer.ptr, &_drv_data->ws_opctx);
  if (ret) return ret;
  dev_info(&spi->dev, "ws2812 init\n");

  dev_set_drvdata(&spi->dev, _drv_data);
  dev_info(&spi->dev, "drv data = %p\n", _drv_data);

  // clear leds
  ws2812_vleds_update(_drv_data);

  _enrty = of_get_child_by_name(spi->dev.of_node, "leds");
  for_each_child_of_node(_enrty, child) {

    // led name
    const char* _label;
    struct led_classdev* _ledcls;
    int _max_brightness;
#ifdef CONFIG_WS2812_VLEDS_CHANNEL_CONTROL
    struct led_classdev* _ledcls_red;
    struct led_classdev* _ledcls_green;
    struct led_classdev* _ledcls_blue;
    char* _red_name;
    char* _green_name;
    char* _blue_name;
#endif
    struct wsled_data* _ledctx;
    const char* _color_value = NULL;
    uint8_t _color_r = 0xff, _color_g = 0xff, _color_b = 0xff;

    if (of_property_read_string(child, "label", &_label)) {
      _label = child->name; // fallback to node name
      dev_warn(&spi->dev, "unamed led, fallback to %s\n", _label);
    }

    // Create main LED device
    _ledcls = devm_kzalloc(&spi->dev, sizeof(*_ledcls), GFP_KERNEL);
    if (!_ledcls) return -ENOMEM;

    // max brightness
    if (of_property_read_s32(child, "max_brightness", &_max_brightness)) {
      _max_brightness = 255; // fallback to 255
    }

    _ledcls->name = _label;
    _ledcls->brightness_set_blocking = __cb_set_wsled;
    _ledcls->max_brightness = _max_brightness;
    _ledcls->dev = &spi->dev;

    led_classdev_register(&spi->dev, _ledcls);
    dev_info(&spi->dev, "registering led: %s\n", _label);

#ifdef CONFIG_WS2812_VLEDS_CHANNEL_CONTROL
    // create RGB sub-devices for this led
    _ledcls_red = devm_kzalloc(&spi->dev, sizeof(*_ledcls_red), GFP_KERNEL);
    _ledcls_green = devm_kzalloc(&spi->dev, sizeof(*_ledcls_green), GFP_KERNEL);
    _ledcls_blue = devm_kzalloc(&spi->dev, sizeof(*_ledcls_blue), GFP_KERNEL);
    if (!_ledcls_red || !_ledcls_green || !_ledcls_blue) return -ENOMEM;

    _red_name = devm_kasprintf(&spi->dev, GFP_KERNEL, "%s:red", _label);
    _green_name = devm_kasprintf(&spi->dev, GFP_KERNEL, "%s:green", _label);
    _blue_name = devm_kasprintf(&spi->dev, GFP_KERNEL, "%s:blue", _label);
    if (!_red_name || !_green_name || !_blue_name) return -ENOMEM;

    // setup red channel led
    _ledcls_red->name = _red_name;
    _ledcls_red->brightness_set_blocking = __cb_set_wsled_red;
    _ledcls_red->max_brightness = 255;
    _ledcls_red->dev = &spi->dev;

    // setup green channel led
    _ledcls_green->name = _green_name;
    _ledcls_green->brightness_set_blocking = __cb_set_wsled_green;
    _ledcls_green->max_brightness = 255;
    _ledcls_green->dev = &spi->dev;

    // setup blue channel led
    _ledcls_blue->name = _blue_name;
    _ledcls_blue->brightness_set_blocking = __cb_set_wsled_blue;
    _ledcls_blue->max_brightness = 255;
    _ledcls_blue->dev = &spi->dev;
#endif

    _ledctx = devm_kzalloc(&spi->dev, sizeof(*_ledctx), GFP_KERNEL);
    if (!_ledctx) return -ENOMEM;

    // led default color
    // NOTE: _color_value is pre-initialized to NULL above. Previously it was
    // left uninitialized when this property lookup failed, and would get
    // passed straight into hexclr_to_rgb888() -> hexclr_validate() as a
    // garbage pointer.
    if (of_property_read_string(child, "color-value", &_color_value)) {
      dev_warn(&spi->dev, "use 0xffffff(white) as default led color\n");
    }

    // parse string color into int
    if(!hexclr_to_rgb888(_color_value, &_color_r, &_color_g, &_color_b)) {
      dev_warn(&spi->dev, "invalid led color format, use 0xffffff\n");
    }

    dev_info(&spi->dev, "led color %s = %d %d %d \n", _color_value,
      _color_r, _color_g, _color_b);

    _ledctx->cls = _ledcls;
    _ledctx->origin_color.r = _color_r;
    _ledctx->origin_color.g = _color_g;
    _ledctx->origin_color.b = _color_b;
    _ledctx->lightness = 0;
    __set_lightness_color24(&_ledctx->origin_color, &_ledctx->color, _ledctx->lightness);

    // register RGB sub-devices after setting initial values
#ifdef CONFIG_WS2812_VLEDS_CHANNEL_CONTROL
    _ledcls_red->brightness = _color_r;
    _ledcls_green->brightness = _color_g;
    _ledcls_blue->brightness = _color_b;
    _ledctx->cls_red = _ledcls_red;
    _ledctx->cls_green = _ledcls_green;
    _ledctx->cls_blue = _ledcls_blue;
    led_classdev_register(&spi->dev, _ledcls_red);
    led_classdev_register(&spi->dev, _ledcls_green);
    led_classdev_register(&spi->dev, _ledcls_blue);
    dev_info(&spi->dev, "registering rgb leds: %s, %s, %s\n", _red_name, _green_name, _blue_name);
#endif

    list_add_tail(&_ledctx->list, &_drv_data->leds);
  }

  return 0;
}

/* 5.10.x spi_driver.remove is `int (*)(struct spi_device *)`, not void
 * (the void-returning remove() only landed in much newer kernels) */
static int ws2812_vleds_remove(struct spi_device *spi) {
  struct driver_data* _drv_data = dev_get_drvdata(&spi->dev);
  struct wsled_data* _node, *_tmp;

  if (_drv_data) {

    // clear leds
    ws2812_clear(_drv_data->ws_opctx, ws2812_rgb(0, 0, 0));
    ws2812_vleds_update(_drv_data);

    list_for_each_entry_safe(_node, _tmp, &_drv_data->leds, list) {
      if (_node->cls) {
        led_classdev_unregister(_node->cls);
      }

#ifdef CONFIG_WS2812_VLEDS_CHANNEL_CONTROL
      if (_node->cls_red) {
        led_classdev_unregister(_node->cls_red);
      }
      if (_node->cls_green) {
        led_classdev_unregister(_node->cls_green);
      }
      if (_node->cls_blue) {
        led_classdev_unregister(_node->cls_blue);
      }
#endif

      list_del(&_node->list);
    }
  }

  pr_info("virtual leds removed for %s\n", dev_name(&spi->dev));

  return 0;
}

static const struct of_device_id match_table[] = {
  { .compatible = "ws2812-vleds" },
  { /* end */ }
};
MODULE_DEVICE_TABLE(of, match_table);

static struct spi_driver ws2812_vleds_driver = {
  .probe = ws2812_vleds_probe,
  .remove = ws2812_vleds_remove,
  .driver = {
    .name = "ws2812-vleds",
    .of_match_table = match_table,
  },
};
module_spi_driver(ws2812_vleds_driver);

MODULE_AUTHOR("HyperPROBE Labs. <hyperprobelabs@proton.me>");
MODULE_DESCRIPTION("ws2812 SPI virtual LED driver");
MODULE_LICENSE("GPL v2");
