# The Murgia Hack microkernel system

This is the Murgia Hack system, a microkernel and OS useful to experiment with hardware, build modular system, and running unikernels (hereinafter "have fun").

The leitmotiv and design principle of the MH system is that "Everything Is A Device".

For more information refer to the [Murgia Hack system home page](http://mhsys.org), or have a look at the [FOSDEM 2016](https://archive.fosdem.org/2016/schedule/event/microkernels_mh_experiment/) and [FOSDEM 2018](https://fosdem.org/2018/schedule/event/microkernel_mh_everything_is_a_device/) presentations.

## Building the MH system.

In order to build MH, you will need GNU make, and a working C compiler in the host. This has been tested in GNU/Linux and OSX.

### Step #1: Compile the toolchain.

Refer to the [MH toolchain README](https://github.com/mhsys/toolchain) for detailed instruction how to download, build and install the compilers.

Be sure that you have in the path the toolchain binaries you've just installed:

	$ export PATH=<INSTALLDIR>/bin:$PATH

Where INSTALLDIR is the installation directory used during the toolchain compilation. A quick way to check this is that the `i686-elf-murgia-gcc` executable can be found by the shell.

### Step #2: Compile the MH system.

Building MH should be straighforward. From the main directory, simply type:

	$ make
	
This will go on for a while, and produce a binary called 'dist/ukern'. This contains both a multiboot binary and a the initial bootstrap server.

## Running the MH system.

The quickest way is to try it with qemu:

	$ qemu-system-x86_64 -kernel dist/ukern

If you want to try it on a real system, you can boot it from grub:

	> multiboot PATH/TO/ukern
	> boot


## FAQ

### What is the status of the project?

The project is under active, albeit slow, devlopment. The kernel is relatively complete, having been the biggest focus of my interest, the userspace has a couple of very useful libraries and is currently being expaned into a more useful tool.

### Did you say unikernels?

Yes, quite pompously. I found the architecture of MH a natural enviroment for running unikernels in userspace. Experiments have been made with [rumprun](https://github.com/glguida/rumprun/tree/mrg), and the future will probably bring more experiment with MirageOS and Solo5.

### What does it do? If nothing, can I expect it to do something useful relatively soon?

Nothing, yes. Current scope is that of getting all the feature that the kernel actually has into something visible.

### What could the current code theoretically do?

The code base is non-trivial. It can enumerates devices present at boot, it supports userspace device drivers for it, the rump kernel modules have been ported so it is possible to use NetBSD drivers, filesystem and other code in userspace to driver real hardware, it has a complete libc in userspace, Newlib, and has a really useful native runtime system, libmrg.

What needs to be done is bringing all these tools togeter. And fix bugs, of course.

### What machines it runs on?

The system has not be ported yet to any architecture other than i586+, despite being designed and developed with portability in mind.

### Who's behind this?

[This person](http://tlbflush.org).

### Can I help?

Most certainly!
