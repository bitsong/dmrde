#

#
#  ======== makefile ========
#


EXBASE = .
include ../products.mak


ifeq (,$(APP_INSTALL_BASE))
APP_INSTALL_BASE=/home/qwc/workdir/nfsroot
endif

#
#your application parameter

RMTHOST			= qwc@192.168.1.116
RMTPSWD			= 123456
LOCALPSWD		= 123456
REMOTE_INSTALL_ROOT 	= $(APP_INSTALL_BASE)/ti_tisdk_rootfs/syslink_test/
REMOTE_INSTALL_DIR  	= syslink

INSTALL_DIR		= /home/qwc/workdir/nfsroot/ti_tisdk_rootfs/syslink_test/syslink
srcs 			?= syslink.c object.c syslink_notifier.c syslink_messageq.c syslink_ringpipe.c syslink_init.c syslink_sync.c  

srcs			+= msgq_init.c
srcs			+= ntf_init.c
srcs			+= rpe_init.c
 
srcs			+= data2571.c 

srcs			+= sample_init.c
srcs			+= sample_omapl138_cfg.c 					
srcs			+= sample_omapl138_int_reg.c
srcs			+= sample_cs.c 	

srcs			+= device_mcbsp.c      	   
srcs			+= mcbsp_drv.c
srcs			+= mcbsp_edma.c
srcs			+= mcbsp_ioctl.c
srcs			+= mcbsp_osal.c

srcs			+= mcbsp_task.c
srcs			+= main_dsp.c


goal 			=  server_dsp


#ifeq (,$(VPATH))
#VPATH = ../shared
#endif

objs = $(addprefix bin/$(PROFILE)/obj/,$(patsubst %.c,%.oe674,$(srcs)))
libs = configuro/linker.cmd


all: configuro/linker.cmd
	$(MAKE) PROFILE=debug $(goal).x
	$(MAKE) PROFILE=release $(goal).x

$(goal).x: bin/$(PROFILE)/$(goal).xe674
bin/$(PROFILE)/$(goal).xe674: $(objs) $(libs)
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	$(LD) $(LDFLAGS) -o $@ $^ $(LDLIBS)

bin/$(PROFILE)/obj/%.oe674: %.h
bin/$(PROFILE)/obj/%.oe674: %.c
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	$(CC) $(CPPFLAGS) $(CFLAGS) --output_file=$@ -fc $<

configuro/linker.cmd: Dsp.cfg config.bld
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	$(XDC_INSTALL_DIR)/xs --xdcpath="$(subst +,;,$(PKGPATH))" \
            xdc.tools.configuro -o configuro \
            -t ti.targets.elf.C674 -c $(CGT_C674_ELF_INSTALL_DIR) \
            -p ti.platforms.evmOMAPL138:dsp -b config.bld \
            -r release Dsp.cfg

install:
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	@$(MKDIR) $(INSTALL_DIR)/debug
	$(CP) bin/debug/$(goal).xe674 $(INSTALL_DIR)/debug
	@$(MKDIR) $(INSTALL_DIR)/release
	$(CP) bin/release/$(goal).xe674 $(INSTALL_DIR)/release

commit:
#	@echo "topdir:$(TOPDIR),scriptsdir:$(SCRIPTS_DIR)"
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	@scpto.sh . $(REMOTE_INSTALL_DIR) 
#	@$(SCRIPTS_DIR)/autoscp.sh bin/debug/$(goal).xe674 $(REMOTE_INSTALL_DIR)/ $(RMTPSWD) $(LOCALPSWD)
#	@$(SCRIPTS_DIR)/autoscp.sh bin/release/$(goal).xe674 $(REMOTE_INSTALL_DIR)/ $(RMTPSWD) $(LOCALPSWD)
sync:
	@echo "#"
	@echo "# Synchronizing exec to windows..."
	@mvtowin.sh bin

help:
	@$(ECHO) "make                   # build executable"
	@$(ECHO) "make clean             # clean everything"

clean::
	$(RMDIR) configuro bin

PKGPATH := $(SYSLINK_INSTALL_DIR)/packages
PKGPATH := $(PKGPATH)+$(BIOS_INSTALL_DIR)/packages
PKGPATH := $(PKGPATH)+$(IPC_INSTALL_DIR)/packages
PKGPATH := $(PKGPATH)+$(XDC_INSTALL_DIR)/packages
PKGPATH := $(PKGPATH)+$(PDK_INSTALL_DIR)/packages
PKGPATH := $(PKGPATH)+$(EDMA3_INSTALL_DIR)/packages
ifneq (,$(SHARED_DIR))
PKGPATH := $(PKGPATH)+$(SHARED_DIR)
endif
#$(warning $(PKGPATH))

#  ======== install validation ========
ifeq (install,$(MAKECMDGOALS))
ifeq (,$(EXEC_DIR))
$(error must specify EXEC_DIR)
endif
endif

#  ======== toolchain macros ========
CGTOOLS = $(CGT_C674_ELF_INSTALL_DIR)

CC = $(CGTOOLS)/bin/cl6x -c --gcc --define=omapl138 --define=MCBSP_LOOP_PING_PONG --define=MCBSP_LOOPJOB_ENABLE
AR = $(CGTOOLS)/bin/ar6x rq
LD = $(CGTOOLS)/bin/lnk6x --abi=eabi 
ST = $(CGTOOLS)/bin/strip6x

CPPFLAGS =
CFLAGS = -qq $(CCPROFILE_$(PROFILE)) -I. $(COMPILER_OPTS) 

LDFLAGS = -w -q -c -m $(@D)/obj/$(@F).map 
LDLIBS = -l $(CGTOOLS)/lib/rts6740_elf.lib	\
		 -l uart1_stdio.lib\
		 -l mcbsp.cmd

CCPROFILE_debug = -D_DEBUG_=1 --symdebug:dwarf
CCPROFILE_release = -O2
COMPILER_OPTS = $(shell cat configuro/compiler.opt)

#  ======== standard macros ========
ifneq (,$(wildcard $(XDC_INSTALL_DIR)/bin/echo.exe))
    # use these on Windows
    CP      = $(XDC_INSTALL_DIR)/bin/cp
    ECHO    = $(XDC_INSTALL_DIR)/bin/echo
    MKDIR   = $(XDC_INSTALL_DIR)/bin/mkdir -p
    RM      = $(XDC_INSTALL_DIR)/bin/rm -f
    RMDIR   = $(XDC_INSTALL_DIR)/bin/rm -rf
else
    # use these on Linux
    CP      = cp
    ECHO    = echo
    MKDIR   = mkdir -p
    RM      = rm -f
    RMDIR   = rm -rf
endif

#  ======== create output directories ========
ifneq (clean,$(MAKECMDGOALS))
ifneq (,$(PROFILE))
ifeq (,$(wildcard bin/$(PROFILE)/obj))
    $(shell $(MKDIR) -p bin/$(PROFILE)/obj)
endif
endif
endif
