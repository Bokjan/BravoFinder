# General
This is a dynamic library based on CMake building system.

# Documentation
Currently not available.

# How to build
## Linux
```bash
$ mkdir build
$ cmake ..
$ make
$ sudo make install
$ sudo ldconfig
```
You may need to add your CMake install directory to `/etc/ld.so.conf` or somewhere else effective before executing `ldconfig`.

After installing, you can build programs with BravoFinder APIs. Don't forget to add linker flag `-lbfinder`.

## Windows
I'm not familiar with Windows developing. Visual Studio + CMake may be a reasonable choice.

Having no Visual Studio, I successfully built this using Dev-C++, a lightweight free software (GNU toolchain). CMake won't work on it, and you need to create a project and add source files manually.

## macOS
This project is developed under macOS. A Mac user knows what to do.
