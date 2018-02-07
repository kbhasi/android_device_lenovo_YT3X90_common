LOCAL_PATH:= $(call my-dir)

common_src_files := \
    idc.c \
    image.c \
    fileio.c \

coff_headers = coff/external.h coff/pe.h

common_ldadd = libccan libcrypto_static2
uuid_libs = libext2_uuid_host

common_cflags := \
    -Wall -Wextra --std=gnu99 -Wno-pointer-arith\
    -pipe -fno-strict-aliasing -Wno-sign-compare  \
    -Wno-unused-parameter \

ifeq ($(HOST_OS),darwin)
common_cflags += -DDARWIN
endif

efi_prebuilts_root := hardware/intel/efi_prebuilts
ifeq ($(HOST_ARCH),x86_64)
    gnuefi_top=$(efi_prebuilts_root)/gnu-efi/linux-x86_64/
else
    gnuefi_top=$(efi_prebuilts_root)/gnu-efi/linux-x86/
endif

uuid_cflags := \
    -Iexternal/e2fsprogs/lib \

efi_cflags := \
    -I$(gnuefi_top)/include \
    -I$(gnuefi_top)/include/efi \
    -I$(gnuefi_top)/include/efi/$(HOST_ARCH) \


common_includes := \
    $(LOCAL_PATH)/ \
    $(LOCAL_PATH)/coff \
    vendor/intel/external/openssl/include \
    $(LOCAL_PATH)/../lib/ccan/ \
    $(LOCAL_PATH)/../ \
    prebuilts/gcc/linux-x86/x86/x86_64-linux-android-4.7/include \


##
# sbsign
#
include $(CLEAR_VARS)

LOCAL_MODULE := sbsign
LOCAL_MODULE_TAGS := optional
LOCAL_C_INCLUDES := $(common_includes)
LOCAL_CFLAGS := $(common_cflags)
LOCAL_SRC_FILES := $(common_src_files) sbsign.c
LOCAL_STATIC_LIBRARIES := $(common_ldadd)
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_LDLIBS := -ldl

include $(BUILD_HOST_EXECUTABLE)

##
# sbverify
#
include $(CLEAR_VARS)

LOCAL_MODULE := sbverify
LOCAL_MODULE_TAGS := optional
LOCAL_C_INCLUDES := $(common_includes)
LOCAL_CFLAGS := $(common_cflags)
LOCAL_SRC_FILES := $(common_src_files) sbverify.c
LOCAL_STATIC_LIBRARIES := $(common_ldadd)
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_LDLIBS := -ldl

include $(BUILD_HOST_EXECUTABLE)

##
# sbattach
#
include $(CLEAR_VARS)

LOCAL_MODULE := sbattach
LOCAL_MODULE_TAGS := optional
LOCAL_C_INCLUDES := $(common_includes)
LOCAL_CFLAGS := $(common_cflags)
LOCAL_SRC_FILES := $(common_src_files) sbattach.c
LOCAL_STATIC_LIBRARIES := $(common_ldadd)
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_LDLIBS := -ldl

include $(BUILD_HOST_EXECUTABLE)


##
# sbvarsign
#
include $(CLEAR_VARS)

LOCAL_MODULE := sbvarsign
LOCAL_MODULE_TAGS := optional
LOCAL_C_INCLUDES := $(common_includes)
LOCAL_CFLAGS :=  $(common_cflags) $(uuid_cflags) $(efi_cflags)
LOCAL_SRC_FILES := $(common_src_files) sbvarsign.c
LOCAL_STATIC_LIBRARIES := $(common_ldadd) $(uuid_libs)
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_LDLIBS := -ldl

include $(BUILD_HOST_EXECUTABLE)


##
# sbsiglist
#
include $(CLEAR_VARS)

LOCAL_MODULE := sbsiglist
LOCAL_MODULE_TAGS := optional
LOCAL_C_INCLUDES := $(common_includes)
LOCAL_CFLAGS := $(common_cflags) $(uuid_cflags) $(efi_cflags)
LOCAL_SRC_FILES := $(common_src_files) sbsiglist.c
LOCAL_STATIC_LIBRARIES := $(common_ldadd) $(uuid_libs)
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_LDLIBS := -ldl

include $(BUILD_HOST_EXECUTABLE)

##
# sbkeysync
#
include $(CLEAR_VARS)

LOCAL_MODULE := sbkeysync
LOCAL_MODULE_TAGS := optional
LOCAL_C_INCLUDES := $(common_includes)
LOCAL_CFLAGS := $(common_cflags) $(uuid_cflags) $(efi_cflags)
LOCAL_SRC_FILES := $(common_src_files) sbkeysync.c
LOCAL_STATIC_LIBRARIES := $(common_ldadd) $(uuid_libs)
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_LDLIBS := -ldl

include $(BUILD_HOST_EXECUTABLE)


