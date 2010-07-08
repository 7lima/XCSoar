# Rules for XCSoarLaunch.dll, the launcher for the PocketPC shell

ifeq ($(HAVE_CE)$(findstring $(TARGET),ALTAIR ALTAIRPORTRAIT),y)

XCSOARLAUNCH_DLL = $(TARGET_BIN_DIR)/XCSoarLaunch.dll
XCSOARLAUNCH_SOURCES = \
	$(SRC)/XCSoarLaunch.c
XCSOARLAUNCH_OBJS = $(call SRC_TO_OBJ,$(XCSOARLAUNCH_SOURCES))
$(XCSOARLAUNCH_OBJS): CFLAGS += -Wno-missing-declarations -Wno-missing-prototypes

$(TARGET_OUTPUT_DIR)/XCSoarLaunch.e: $(SRC)/XCSoarLaunch.def $(XCSOARLAUNCH_OBJS) | $(TARGET_BIN_DIR)/dirstamp
	$(Q)$(DLLTOOL) -e $@ -d $^

$(TARGET_OUTPUT_DIR)/XCSoarLaunch.rsc: Data/XCSoarLaunch.rc | $(TARGET_OUTPUT_DIR)/dirstamp
	@$(NQ)echo "  WINDRES $@"
	$(Q)$(WINDRES) $(WINDRESFLAGS) -o $@ $<

$(XCSOARLAUNCH_DLL): TARGET_LDLIBS = -laygshell
$(XCSOARLAUNCH_DLL): $(TARGET_OUTPUT_DIR)/XCSoarLaunch.e $(XCSOARLAUNCH_OBJS) $(TARGET_OUTPUT_DIR)/XCSoarLaunch.rsc | $(TARGET_BIN_DIR)/dirstamp
	$(CC) -shared $(LDFLAGS) $(TARGET_ARCH) $^ $(LOADLIBES) $(LDLIBS) -o $@

else

XCSOARLAUNCH_DLL =

endif
