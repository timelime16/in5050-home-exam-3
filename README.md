# Codec63 #

## Dolphin SISCI

This branch contains a few changes compared to master to facilitate using SISCI
in a Dolphin NTB cluster. First of all, three new executables are build:
c63server, c63worker and c63writer. The writer is a copy of c63enc with some code
removed and some additional arguments. All the new executables parse arguments,
including the nodeids of the other nodes and call `SCIInitialize`. Otherwise
they are pretty empty, and you need to fill in the blanks. The intention is that
c63server reads the inputfile, c63worker encodes the video and c63writer receives
and writes the encoded video to disk.
Additionally there is a `run.sh` script that compiles and runs the code
on all 4 target nodes. This script should be run on the login node, it
automatically compiles and copies over the binaries to the nodes, and runs, passing
remote node ids as arguments to that you can boot strap communication.

After the run has completed, the script writes logfiles containing the output
from each program and copies the output video file back to the source directory.

You may change the `run.sh` script, but the delivery should contain the
modified run.sh and it should require no additional arguments.

For this exam, the cuda version has been removed.

### Things you need to change

- Set your group number in `c63.h`


## Usage

The `run.sh` script will compile and launch the encoder on all nodes.  To
specify which 4 nodes to run on, specify the `--block` parameter. This selects
a group of 4 nodes (hard coded in the script). Run the script without arguments to
see the usage.

Example usage:
```
# ./run.sh --block 1

Source dir: /home/larsbk/in5050-codec63/c63-in-c
### Compiling on localhost ###

(...)

[ 30%] Built target c63

(...)

### Copying over binaries to remote hosts ###
Server dolphin-01 command: c63server  -r 8 -r 12 -o 16 -w 352 -h 288 /opt/Media/foreman.yuv
Writer dolphin-04 command: c63writer -s 4  -r 8 -r 12 -o ./output.c63
Worker dolphin-02 command: c63worker -s 4  -r 8 -r 12
Worker dolphin-03 command: c63worker -s 4  -r 8 -r 12

### Running ###

(...)
server: Hello World!

(...)
Copying back output file from writer node
-rw-rw-r-- 1 larsbk larsbk 6 april 28 15:47 /home/larsbk/in5050-codec63/output.c63

(...)
Logfiles:
-rw-rw-r-- 1 larsbk larsbk  21 april 28 15:47 logs/20260428-134747-dolphin-02.log
-rw-rw-r-- 1 larsbk larsbk  21 april 28 15:47 logs/20260428-134747-dolphin-03.log
-rw-rw-r-- 1 larsbk larsbk 166 april 28 15:47 logs/20260428-134747-server.log
-rw-rw-r-- 1 larsbk larsbk  21 april 28 15:47 logs/20260428-134747-writer.log
```

We strongly recommend that clone the github repository in5050-codec63 twice.
Make your change in one of them, but use the c53dec and c63pred command from your
_other_ checkout. The reason is that nearly all files are shared between encoding
and decoding.

To decode the c63 file:
```
./c63dec foremanout.c63 output.yuv
```

To playback the raw YUV file before encoding or after decoding, you can use mplayer or ffplay.
```
mplayer -demuxer rawvideo -rawvideo w=352:h=288 output.yuv
ffplay -f rawvideo -pixel_format yuv420p -video_size 352x288 -i output.yuv
```
When you are remotely logged into a Tegra, you may have to force the use of X11:
```
mplayer -vo x11 -demuxer rawvideo -rawvideo w=352:h=288 output.yuv
```


### Description ###
This project is used in IN5050 (Programming Heterogeneous Multi-core Architectures) at the Department of Informatics, University of Oslo, Norway. For more information, see the [course page](https://www.uio.no/studier/emner/matnat/ifi/IN5050/).
