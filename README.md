# argos-experiments
## argos3 + this project Installation instruction on Fedora 39

install argos dependencies:   `sudo dnf install glew libGLEW glew-devel freeglut-devel glm-devel cmake g++ qt5-qtbase-devel.x86_64 lua lua-devel doxygen`


get and compile argos3:

git clone https://github.com/ilpincy/argos3.git argos3

cd argos3

mkdir build_simulator

cd build_simulator

cmake ../src

make

in the project's directory:

(beware that your prefix in the argos3.pc file has to be /usr/local)

`sudo cp argos3.pc /usr/share/pkgconfig`

`meson setup builddir`

`ninja -C builddir`

launch the project with: `argos3 -c conf/meson.argos`
