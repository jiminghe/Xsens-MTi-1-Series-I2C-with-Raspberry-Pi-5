# OBJLIBS	= libxscontroller libxscommon libxstypes
OBJLIBS	= libmtinterface libxbus libxstypes
all : $(OBJLIBS)

libxstypes :
	$(MAKE) -C xstypes $(MFLAGS) libxstypes.a

libxbus :
	$(MAKE) -C xbus $(MFLAGS) libxbus.a

libmtinterface :
	$(MAKE) -C mtinterface $(MFLAGS) libmtinterface.a

clean :
	-$(MAKE) -C xstypes $(MFLAGS) clean
	-$(MAKE) -C mtinterface $(MFLAGS) clean
	-$(MAKE) -C xbus $(MFLAGS) clean
