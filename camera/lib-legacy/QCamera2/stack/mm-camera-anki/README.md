# mm-camera-anki

Sorting out some of the more techincial stuff going on here.

## Motivation

The docs indicate that for some unexplained reason vic-engine could
not talk directly to the camera because it used the android toolchain
to build. As a result this was developed as a shim. It talks to the
camera, and vic-engine talks directly to it.

## ION Memory

This is essentially and Android Direct Memory Access feature. The
images are so large we don't want to send them over the control
socket. `vic-engine` reads directly on the other side.

## Debayer

This code is a bit confusing and includes an optimized assembly
version. Lets take a step back and explain it. To begine we'll assume
we're using the original Vector 1.0 camera sensor which is one
megapixel with a resolution of 1280x720.

First note that in a camera sensor, each 'pixel' is a covered with a
mask and a single color, either Red, Green or Blue. They are
physically arranged in a BGGR format, and 10 bits.

For pixels on the sensor:

```
+-----+
| B G |
| G R |
+-----+
```

But we want a traditional RGB pixel for use by the other systems.

These are also 10 bit numbers, so we need to change them to 8 bit to
get an RGB pixel:

[(R >> 2), ((G1 + G2) >> 4), (B >> 2)]

Since there are 2 bit pixels per number, and the arrangement requires
two rows, the debayer processes 4 columns of pixels and two rows per
its inner loop:

```
+-----+-----+
| B G | B G |
| G R | G R |
+-----+-----+
```

We need the two rows to get a full RGB color as shown above. But why
four columns? Because 4 columns of 10 bit data is 40 bits, or 5 bytes
which is evenly aligned in memory.  This allows us to use the same
code in the inner loop for each pass and not have to deal with a half
byte odd-row/even-row approach when going through the loop keeping
things simple.

When we go through this process on the original sensor which has
1280x720 monochrome pixels, we end up with a set of RGB pixels that is
640x360. This is where the original image size comes from.

Now that we're upgrading to a 2 megapixel camera with resolution of
1600x1200 the initially converted version has a size of 800x600. Now
we need to decide if this has additional downsampling to 640x360, and
if that happens here.

## Testing

1. Make the following change so we save debug images:

    ```
    diff --git a/camera/lib-legacy/QCamera2/stack/mm-camera-anki/src/camera_process.c b/camera/lib-legacy/QCamera2/stack/mm-camera-anki/src/camera_process.c
    index f1fda89..05229ec 100644
    --- a/camera/lib-legacy/QCamera2/stack/mm-camera-anki/src/camera_process.c
    +++ b/camera/lib-legacy/QCamera2/stack/mm-camera-anki/src/camera_process.c
    @@ -26,7 +26,7 @@
     #include "camera_memory.h"
      #include "log.h"

    -static int  s_dump_images = 0;
    +static int  s_dump_images = 1;
     static char s_output_path[256] = "/data/misc/camera/test";

     // Camera Memory
     ```

1. On the robot stop the existing processes `systemctl stop vic-engine mm-anki-camera`

1. On the robot make sure the test directory exists: `mkdir -p /data/misc/camera/test`

1. On the robot run `/usr/bin/mm-anki-camera -C -d -o /data/misc/camera/test`

1. Let it run for a bit then `Ctrl-C`

1. There should now be RGB image files in the test directory. Use a rgb
    viewer such as https://rawpixels.net/ to view.
