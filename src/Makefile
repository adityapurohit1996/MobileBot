#ly add your directories to this folder if you are 100% sure that
# it will always compile without warnings.
# NOTE: REMOVED planning FROM SUBDIRS FOR TESTING ADD IT COMPILING ISSUES 
SUBDIRS = lcmtypes common imagesource vx/math vx vx/gtk slam apps mbot optitrack
MBOT_SUBDIRS = lcmtypes common vx/math slam mbot
LAPTOP_SUBDIRS = lcmtypes common imagesource vx/math vx vx/gtk planning slam apps

MAKEFLAGS += --no-print-directory

laptop-only:
	@$(MAKE) -f Makefile.lcmgen all || exit 2;
	@for dir in $(LAPTOP_SUBDIRS); do \
	echo "[$$dir]"; $(MAKE) -C $$dir all || exit 2; done	

mbot-only:
	@$(MAKE) -f Makefile.lcmgen all || exit 2;
	@for dir in $(MBOT_SUBDIRS); do \
	echo "[$$dir]"; $(MAKE) -C $$dir all || exit 2; done

all:
	@$(MAKE) -f Makefile.lcmgen all || exit 2;
	@for dir in $(SUBDIRS); do \
	echo "[$$dir]"; $(MAKE) -C $$dir all || exit 2; done

clean:
	@for dir in $(SUBDIRS); do \
	echo "clean [$$dir]"; $(MAKE) -C $$dir clean || exit 2; done
	@rm -f *~

