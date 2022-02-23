ifeq (,$(strip $(LOCAL_MODULE_SRC_PATH)))
  $(error Variable LOCAL_MODULE_SRC_PATH is empty)
endif

# Assign external kernel modules to the DLKM class
LOCAL_MODULE_CLASS := DLKM

# Get rid of any whitespace
LOCAL_MODULE_KBUILD_NAME := $(strip $(LOCAL_MODULE_KBUILD_NAME))

# Set default output module path
LOCAL_MODULE_PATH := $(strip $(LOCAL_MODULE_PATH))
LOCAL_MODULE_PATH := $(if $(LOCAL_MODULE_PATH),$(LOCAL_MODULE_PATH),$(KERNEL_MODULES_OUT))

include $(BUILD_SYSTEM)/base_rules.mk

LOCAL_MODULE_KBUILD_INTERMEDIATES_NAME := $(TARGET_OUT_INTERMEDIATES)/$(LOCAL_MODULE_SRC_PATH)/$(LOCAL_MODULE_KBUILD_NAME)

# Rule install kernel module
$(LOCAL_BUILT_MODULE): $(LOCAL_MODULE_KBUILD_INTERMEDIATES_NAME) | $(ACP)
ifneq ($(LOCAL_MODULE_DEBUG_ENABLE),true)
	$(hide) mkdir -p $(dir $@)
	$(hide) cp $< $<.unstripped
	$(hide) $(TARGET_STRIP) --strip-debug $<
	$(hide) cp $< $<.stripped
endif
	@sh -c "\
	   KMOD_SIG_ALL=`cat $(KERNEL_OUT)/.config | grep CONFIG_MODULE_SIG_ALL | cut -d'=' -f2`; \
	   KMOD_SIG_HASH=`cat $(KERNEL_OUT)/.config | grep CONFIG_MODULE_SIG_HASH | cut -d'=' -f2 | sed 's/\"//g'`; \
	   if [ \"\$$KMOD_SIG_ALL\" = \"y\" ] && [ -n \"\$$KMOD_SIG_HASH\" ]; then \
	      echo \"Signing kernel module: \" `basename $<`; \
	      cp $< $<.unsigned; \
	      $(MODULE_SIGN_FILE) \$$KMOD_SIG_HASH $(MODSECKEY) $(MODPUBKEY) $<; \
	   fi; \
	"
	$(transform-prebuilt-to-target)

# Dependency to build kernel module
$(LOCAL_MODULE_KBUILD_INTERMEDIATES_NAME): $(LOCAL_MODULE_SRC_PATH)

# This should really be cleared in build/core/clear-vars.mk, but for
# the time being, we need to clear it ourselves
LOCAL_MODULE_KBUILD_NAME :=
LOCAL_MODULE_KBUILD_INTERMEDIATES_NAME :=
LOCAL_MODULE_SRC_PATH :=
