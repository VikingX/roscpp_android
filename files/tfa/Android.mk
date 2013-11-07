LOCAL_PATH := $(call my-dir)

# These must go in some sort of order like include flags, otherwise they are dropped
# Oh no, need to automate this with catkin somehow....
stlibs := roscpp xmlrpcpp rosconsole cpp_common console_bridge boost_date_time boost_filesystem boost_program_options
stlibs += boost_regex boost_system boost_signals boost_thread 
stlibs += roscpp_serialization rostime

shlibs := $(stlibs)

define include_shlib
$(eval include $$(CLEAR_VARS))
$(eval LOCAL_MODULE := $(1))
$(eval LOCAL_SRC_FILES := $$(LOCAL_PATH)/lib/lib$(1).so)
$(eval include $$(PREBUILT_SHARED_LIBRARY))
endef
define include_stlib
$(eval include $$(CLEAR_VARS))
$(eval LOCAL_MODULE := $(1))
$(eval LOCAL_SRC_FILES := $$(LOCAL_PATH)/lib/lib$(1).a)
$(eval include $$(PREBUILT_STATIC_LIBRARY))
endef

#$(foreach shlib,$(shlibs),$(eval $(call include_shlib,$(shlib))))
$(foreach stlib,$(stlibs),$(eval $(call include_stlib,$(stlib))))

include $(CLEAR_VARS)
LOCAL_MODULE    := roscpp_android_ndk
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_EXPORT_CPPFLAGS := -fexceptions -frtti
#LOCAL_SRC_FILES := dummy.cpp
#LOCAL_EXPORT_LDLIBS := $(foreach l,$(shlibs),-l$(l)) -L$(LOCAL_PATH)/lib
#LOCAL_EXPORT_LDLIBS := -lstdc++ #-L$(LOCAL_PATH)/lib
#LOCAL_SHARED_LIBRARIES := $(shlibs)
LOCAL_STATIC_LIBRARIES := $(stlibs) gnustl_static

include $(BUILD_STATIC_LIBRARY)
