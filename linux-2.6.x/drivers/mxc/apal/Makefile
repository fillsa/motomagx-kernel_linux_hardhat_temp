# Copyright (C) 2007  Motorola, Inc.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as 
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  
# 02111-1307, USA
#

#
# Makefile for the kernel APAL (Audio Platform Abstraction Layer) driver.
#
# ChangeLog:
# Date (mm/dd/yyyy)        Author        Comment
# 04/27/2007               Motorola      Remove hardcode.
# 08/04/2007               Motorola      Clearmake compliance.
# 09/04/2007               Motorola      Miniplatform build compliance.

# Source in the top-level make options for MOT build environment
include $(BOOTSTRAP)
ifneq ($(HW_ARCH),i686)

# Source in the CKbuild build system to achieve clearmake compliance
include ${STD_DIRPATH}/ckbuild/ckbuild.mk
include ${STD_DIRPATH}/ckbuild/modkconfig.mk

KERNEL_VERSION  	:= 2.6.10_dev
PRODUCT_ROOTFS  	:= $(BUILDTOP)/$(PROC_FAMILY)/$(PROC_TYPE)/fs/
KMODULES_DIR    	:= ${PRODUCT_ROOTFS}/lib/modules/${KERNEL_VERSION}/

CKBUILD_EXTRA_CONFIG = $(shell $(MODKCONFIG) path config)

CKBUILD_EXTRA_ARGS  	+= INSTALL_MOD_PATH=$(BUILDTOP)
CKBUILD_EXTRA_ARGS  	+= CKBUILD_COMP_TOP=$(COMPTOP)

CKBUILD_EXTRA_CFLAGS 	= -DMODKCONFIG -I$(STD_INCPATH) -I$(KERNELDIR)/include

# now finally do the build and install
api_build: $(PROPFILES)
impl: modules modules_install

distclean: clean
	$(KERNEL_MAKE) clean

endif # ifneq ($(HW_ARCH),i686)