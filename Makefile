@all:
	meson setup builddir
	ninja -C builddir

gui:
	argos3 -c conf/meson.argos

nox:
	rm -rf out
	argos3 -c conf/meson.argos -z

clean:
	rm -rf builddir

docs: doc/html
	make -C doc

launch:
	python3 -m pitone random-choice
