LOCAL_PATH := $(call my-dir)
COMMON_SRC_FILES := src/boot.c \
                    src/check.c \
                    src/common.c \
                    src/fat.c \
                    src/file.c \
                    src/io.c \
                    src/lfn.c \
                    src/charconv.c \

OPTFLAGS = -fomit-frame-pointer -D_GNU_SOURCE
WARNFLAGS = -Wextra -Wno-sign-compare -Wno-missing-field-initializers -Wmissing-prototypes -Wstrict-prototypes
CFLAGS += $(OPTFLAGS) $(WARNFLAGS)
#
# mkdosfs
#
include $(CLEAR_VARS)
LOCAL_MODULE := mkdosfs
LOCAL_MODULE_TAGS := optional
LOCAL_SRC_FILES := src/mkfs.fat.c
LOCAL_CFLAGS := $(CFLAGS)
include $(BUILD_HOST_EXECUTABLE)

#
# dosfsck
#
include $(CLEAR_VARS)
LOCAL_MODULE := dosfsck
LOCAL_MODULE_TAGS := optional
LOCAL_SRC_FILES := $(COMMON_SRC_FILES) src/fsck.fat.c
LOCAL_CFLAGS := $(CFLAGS)
include $(BUILD_HOST_EXECUTABLE)
