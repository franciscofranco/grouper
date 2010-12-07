zreladdr-$(CONFIG_ARCH_TEGRA_2x_SOC)	:= 0x00008000
params_phys-$(CONFIG_ARCH_TEGRA_2x_SOC)	:= 0x00000100
initrd_phys-$(CONFIG_ARCH_TEGRA_2x_SOC)	:= 0x00800000

zreladdr-$(CONFIG_ARCH_TEGRA_3x_SOC)	:= 0x80008000
params_phys-$(CONFIG_ARCH_TEGRA_3x_SOC)	:= 0x80000100
initrd_phys-$(CONFIG_ARCH_TEGRA_3x_SOC)	:= 0x80800000

dtb-$(CONFIG_MACH_HARMONY) += tegra-harmony.dtb
dtb-$(CONFIG_MACH_SEABOARD) += tegra-seaboard.dtb
