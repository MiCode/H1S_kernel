#ifndef _LINUX_FBCON_H
#define _LINUX_FBCON_H

#ifdef CONFIG_FRAMEBUFFER_CONSOLE
void __init fb_console_init(void);
void __exit fb_console_exit(void);
int  fbcon_modechange_possible(struct fb_info *info,
			       struct fb_var_screeninfo *var);
#else
static inline void fb_console_init(void) {}
static inline void fb_console_exit(void) {}
static inline int  fbcon_modechange_possible(struct fb_info *info,
				struct fb_var_screeninfo *var) { return 0; }
#endif

#endif /* _LINUX_FBCON_H */
