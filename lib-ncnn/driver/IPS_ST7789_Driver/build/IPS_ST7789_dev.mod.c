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
	{ 0xb497a61, "spi_unregister_device" },
	{ 0xbc20e6da, "device_find_child" },
	{ 0xedf8d420, "device_for_each_child" },
	{ 0xa61db944, "put_device" },
	{ 0x62aa5d1c, "spi_new_device" },
	{ 0x7c32d0f0, "printk" },
	{ 0x84f49cd7, "spi_busnum_to_master" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

