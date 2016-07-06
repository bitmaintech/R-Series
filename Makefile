#
# Author: Xiangfu Liu <xiangfu@openmobilefree.net>
# Address: 12h6gdGnThW385JaX1LRMA8cXKmbYRTP8Q
#
# This is free and unencumbered software released into the public domain.
# For details see the UNLICENSE file at the root of the source tree.
#

include $(TOPDIR)/rules.mk

MINER_TYPE:=R2

PKG_NAME:=cgminer
PKG_VERSION:=4.8.0
PKG_REV:=d5f61e9a5dcc1b1043cd4ae95f64065422f828f2
PKG_RELEASE:=1
PKG_INSTALL:=1

PKG_SOURCE:=$(PKG_NAME)-$(PKG_VERSION).tar.bz2
PKG_SOURCE_SUBDIR:=$(PKG_NAME)-$(PKG_VERSION)
PKG_SOURCE_VERSION:=$(PKG_REV)

PKG_FIXUP:=autoreconf

include $(INCLUDE_DIR)/package.mk

define Package/cgminer
	SECTION:=utils
	CATEGORY:=Utilities
	TITLE:=cgminer (FPGA Miner)
	URL:=https://github.com/ckolivas/cgminer
	DEPENDS:=+libncurses +libcurl +libpthread +jansson +udev 
endef

define Package/cgminer/description
Cgminer is a multi-threaded multi-pool GPU, FPGA and CPU miner with ATI GPU
monitoring, (over)clocking and fanspeed support for bitcoin and derivative
coins. Do not use on multiple block chains at the same time!
endef

CONFIGURE_ARGS += --disable-opencl --disable-adl --disable-nurses --enable-antrouter   --disable-libcurl 
TARGET_LDFLAGS += -Wl,-rpath-link=$(STAGING_DIR)/usr/lib

define Build/Compile
	$(call Build/Compile/Default)
	(cd $(PKG_BUILD_DIR) && \
	  $(TARGET_CC) -Icompat/jansson-2.6/src -Icompat/libusb-1.0/libusb \
	  api-example.c -o cgminer-api;)
endef


define Package/cgminer/install
	$(INSTALL_DIR) $(1)/usr/bin $(1)/etc/init.d
	$(INSTALL_DIR) $(1)/etc/config

	$(INSTALL_BIN) $(PKG_BUILD_DIR)/cgminer-api $(1)/usr/bin/cgminer-api

	$(INSTALL_BIN) $(PKG_INSTALL_DIR)/usr/bin/cgminer $(1)/usr/bin/cgminer
	$(INSTALL_BIN) $(FILES_DIR)/cgminer-monitor       $(1)/usr/bin
	$(INSTALL_BIN) $(FILES_DIR)/self-checking         $(1)/usr/bin
	$(INSTALL_BIN) $(FILES_DIR)/cgminer.init          $(1)/etc/init.d/cgminer
	$(INSTALL_BIN) $(FILES_DIR)/usr_bak          $(1)/etc/init.d/usr_bak

	$(CP) $(FILES_DIR)/usr.config $(1)/etc/config/cgminer

	rm -rf $(1)/usr/bin/compile_time
	date >> $(1)/usr/bin/compile_time
	echo $(MINER_TYPE)
	echo $(MINER_TYPE) >> $(1)/usr/bin/compile_time
endef

$(eval $(call BuildPackage,cgminer))
