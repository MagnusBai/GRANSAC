# GRANSAC: Multi-threaded generic RANSAC implemetation

This is a header-only, multi-threaded implementation of the [RANSAC algorithm](https://en.wikipedia.org/wiki/RANSAC),
used widely in computer vision.

Unlikely most other implementations, this is a **generic** implementation
which can be adopted for any problem. The user has to implement a class that
inherits the AbstractModel class. Using RANSAC afterwards should just work.

## Dependencies

This library uses *C++11* features, so a suitable compiler is required (GCC 4.7+, 
Visual Studio 2013+). Additionally, *OpenMP* is needed for multi-threading.

Optionally, to build the line fiting example, *OpenCV* and *CMake* are required.

## Usage

### building your own RANSAC application

Just include the header GRANSAC.hpp in your application. The AbstractModel class
needs to be inherited to implement a suitable model for your application.

### using our samples

the following commands will create binary of samples:

```
cd $(GRANSAC_HOME)
mkdir build && cd build
cmake ..
make
```

#### Sample0 : Line Fitting

Running **sample_0_fitting_line** should generate random 2D points around the diagonal and get the line which would fit the distribution best through RANSAC algorithm.

![sample0img](https://imgsa.baidu.com/forum/w%3D580/sign=706030e210d8bc3ec60806c2b289a6c8/7fb847224f4a20a45ac2b91f99529822700ed0bc.jpg)


#### Sample1 : Line Fitting

Running **sample_1_fitting_lines** should load the contour of pre-processed sentences segmentation and get the line which would fit the distribution best through RANSAC algorithm.

![sample1img](https://imgsa.baidu.com/forum/w%3D580/sign=c5d864ef3cfae6cd0cb4ab693fb10f9e/919a7382b2b7d0a25e078145c2ef760949369a4e.jpg)


#### Sample2 : Vanishing Point Estimation

Running **sample_2_vanishing_point** should load some lines and get the vanishing point of them (the most probable intersection point of these lines, as much as better) through RANSAC algorithm.

![sample2img](https://imgsa.baidu.com/forum/w%3D580/sign=c21be2abf2f2b211e42e8546fa816511/4f5ea8025aafa40f99d84a05a264034f79f0197e.jpg)

## License

GRANSAC is released under an [MIT License](https://opensource.org/licenses/MIT).

## Contact

Srinath Sridhar (srinaths@umich.edu)
Max Planck Institute for Informatics
