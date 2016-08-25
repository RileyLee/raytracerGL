Raytracer Comparison Tool
----------------

Compare is a simple program that generates a series of images from your raytracer binary and the solution binary and runs a diff on them.

To use it, get a command prompt and type in:

  compare <sample-exe> <your-exe> [opts]

where <sample-exe> is the filename of the sample solution, and likewise for <your-exe>. (actually including ".exe" is optional)

There are a couple of options:
	-c <dir>	Change the default ("cache") directory where the intermediate images are stored
	-w n		Uses images of size 200 <= n <= 512
	-r		Uses a random image size between 200 and 512

It generates images for the following test scenes, which cover all the project requirements:

simple\box_dist_atten.ray
simple\sphere_refract.ray
simple\cylinder_refract.ray
simple\box_cyl_transp_shadow.ray
simple\box_cyl_reflect.ray
trimeshes\box_sphere.ray

Each scene is rendered sequentially at a depth from 0 to 3. Each of these output images is then compared against the solution by calculating the mean-square deviation. This information is output to stdout. It also produces "diff" images that highlight areas where the prerendered images differed. Files are named "<ray-file>.{sample,student,diff}.dM.wN.png", where M is the depth and N is the image width.

All the deviations are printed to stdout, and (after all tests are run) to results.txt.

Be warned: this counts _any_ deviation at all, even if it's only off by one. If you don't produce exactly the same image, don't worry. Rounding errors do crop up, and you won't be docked for those. This is just to highlight potential problem areas.

If you would like to run other scenes against the solution, just edit the compare.rc file. The first line of the file tells the program which directory contains the scenes, and the remaining lines instruct the program which scenes to use.


NOTE:
The program won't work if any of the files are directly off of the network; it must be on a mapped network drive or local drive. That means you can't tell it to use \\ntdfs\cs\unix\..... for the scenes. Instead, use a mapped drive like O:\, or better yet, copy the entire folder contents onto a local drive and drop a copy of your binary right into the folder. Don't forget to compile in release mode to trim down the execution time.