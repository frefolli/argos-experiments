@all:
	meson setup builddir
	ninja -C builddir
	argos3 -c conf/meson.argos
