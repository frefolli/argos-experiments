@all:
	meson setup builddir
	ninja -C builddir

gui:
	argos3 -c conf/meson.argos

nox:
	argos3 -c conf/meson.argos -z

clean:
	rm -rf builddir
