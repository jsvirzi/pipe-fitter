LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE    := PipeFitter 
LOCAL_SRC_FILES := main.c
LOCAL_LDLIBS    := -llog -landroid

include $(BUILD_EXECUTABLE)
