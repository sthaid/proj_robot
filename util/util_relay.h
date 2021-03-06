#ifndef __UTIL_RELAY_H__
#define __UTIL_RELAY_H__

#define RELAY_TEST  5

static inline int relay_init(void)
{
    set_gpio_func(5, FUNC_OUT);
    return 0;
}

static inline void relay_ctrl(int relay_id, bool enable)
{
    gpio_write(relay_id, enable);
}

#endif
