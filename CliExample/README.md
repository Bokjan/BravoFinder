# Build

## Prerequisite

Build `Library` first.

## *nix

```bash
$ c++ -o bf bf.cpp -std=c++11 -lbfinder
```
## Windows

Set include directory (to header files) properly and dynamically link to DLL file you've built. When doing binary distributing, DLL is required to go with EXE.

# Usage

## Configuration

There should be a `navdata.txt` file in the directory which contains the executable. `navdata.txt` contains the absolute path of PMDG navigation data directory, that is, the directory contains `NAVDATA` and `SIDSTARS`.

## Run

When running, there will be several prompts. The input (ICAO code) is case-insensitive.