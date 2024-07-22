@all:
	meson setup builddir
	ninja -C builddir

gui:
	ninja -C builddir
	argos3 -c conf/meson.argos

nox:
	ninja -C builddir
	rm -rf out
	argos3 -c conf/meson.argos -z

clean:
	rm -rf builddir

docs: doc/html
	make -C doc

launch:
	python3 -m launcher
