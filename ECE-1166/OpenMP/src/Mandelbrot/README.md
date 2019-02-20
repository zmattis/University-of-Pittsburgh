
# MandelbrotSet

This repository contains a simple .ppm image Mandelbrot fractal generator implemented in C.


## Build

This application can be built by simply running the ```make``` command from within this directory.

An alternative build script is included for systems without make, and can be run with '''build.sh'''.


## Usage

The Mandelbrot set application can be run using the following command.

```./mandelbrot <output> <x/y> <display>```

The parameters that follow apply.

+ Output  - a .ppm image to output with the fractal.
+ X/Y     - width and height of image in pixels.
+ Display - 1 displays debug text, 0 just displays time values for raw data tables.

Example:  ```./mandelbrot output.ppm 2048 1```


## Conversion

One may convert from .ppm images using the Unix terminal command:

```convert <filenameIn>.<extensionIn> <filenameOut>.<extensionOut>```

A .ppm file can be opened in Windows using the free GIMP or Inkscape graphics applications.



       - EWG   SDG
