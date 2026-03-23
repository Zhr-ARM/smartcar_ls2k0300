#include <linux/build-salt.h>
#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0xe3ae0b4d, "module_layout" },
	{ 0xf9a482f9, "msleep" },
	{ 0x9c1e5bf5, "queued_spin_lock_slowpath" },
	{ 0x32ac71b8, "spi_sync" },
	{ 0xfb578fc5, "memset" },
	{ 0xa202a8e5, "kmalloc_order_trace" },
	{ 0x93fca811, "__get_free_pages" },
	{ 0x772a8264, "cdev_add" },
	{ 0xc382d164, "cdev_init" },
	{ 0x368ad888, "device_create" },
	{ 0x4c989a74, "__class_create" },
	{ 0xe3ec2f2b, "alloc_chrdev_region" },
	{ 0x3d7cc3be, "spi_setup" },
	{ 0xcdbf1cc, "ioremap" },
	{ 0xd438ff09, "driver_unregister" },
	{ 0xb148aa0f, "__spi_register_driver" },
	{ 0x4302d0eb, "free_pages" },
	{ 0x2f9bd219, "_dev_info" },
	{ 0x37a0cba, "kfree" },
	{ 0xe7043306, "iounmap" },
	{ 0x6091b333, "unregister_chrdev_region" },
	{ 0xe07aac8e, "class_destroy" },
	{ 0x72d2e962, "device_destroy" },
	{ 0xd239f36b, "cdev_del" },
	{ 0x7c32d0f0, "printk" },
	{ 0xe526f136, "__copy_user" },
	{ 0x33dce5f6, "_dev_err" },
	{ 0x7e92cf8b, "remap_pfn_range" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

MODULE_ALIAS("of:N*T*Cloongson,ips-st7789-driver");
MODULE_ALIAS("of:N*T*Cloongson,ips-st7789-driverC*");
