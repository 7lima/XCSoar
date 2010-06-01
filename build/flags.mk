CXX_FEATURES = -fno-exceptions 

ifneq ($(TARGET),WINE)
C_FEATURES = -std=gnu99
else
# libwine fails with -std=gnu99 due to funny "extern inline" tricks in
# winnt.h
C_FEATURES =
endif

GIT_COMMIT_ID := $(shell git rev-parse --short --verify HEAD 2>/dev/null)
ifneq ($(GIT_COMMIT_ID),)
CPPFLAGS += -DGIT_COMMIT_ID=\"$(GIT_COMMIT_ID)\"
endif

ALL_CPPFLAGS = $(TARGET_INCLUDES) $(INCLUDES) $(TARGET_CPPFLAGS) $(CPPFLAGS)
ALL_CXXFLAGS = $(OPTIMIZE) $(FLAGS_PROFILE) $(CXX_FEATURES) $(CXXFLAGS)
ALL_CFLAGS = $(OPTIMIZE) $(FLAGS_PROFILE) $(C_FEATURES) $(CFLAGS)
