@all:
	meson setup builddir
	ninja -C builddir

run:
	argos3 -c conf/meson.argos
