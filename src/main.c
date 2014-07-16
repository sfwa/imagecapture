#include <errno.h>
#include <asm/termios.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdint.h>
#include "chameleon_util.h"

#define RD_BUFSIZE 256u
#define WR_BUFSIZE 256u

#define IMG_WIDTH 1280u
#define IMG_HEIGHT 960u

int set_interface_attribs(int fd, int speed) {
    struct termios2 tty;
    memset(&tty, 0, sizeof tty);

    ioctl(fd, TCGETS2, &tty);
    tty.c_cflag &= ~CBAUD;
    tty.c_cflag |= BOTHER;
    tty.c_ispeed = speed;
    tty.c_ospeed = speed;

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CRTSCTS;

    tty.c_iflag &= ~(IGNBRK | ICRNL | IMAXBEL | BRKINT);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 255u;
    tty.c_cc[VTIME] = 0u;

    ioctl(fd, TCSETS2, &tty);
    return 0;
}

int main(int argc, char** argv) {
    uint32_t camera_brightness = 100;
    uint32_t camera_framerate = 2;
/*
    if (argc < 2) {
        printf("Usage: fcsdump /PATH/TO/DIR");
        return 1;
    }

    char *fname = tempnam(argv[1], "fcs-");
    FILE *f1;
    f1 = fopen(fname, "wb");

    int ifd1 = open("/dev/ttySAC0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (ifd1 < 0) {
        printf("error %d opening /dev/ttySAC0: %s", errno, strerror(errno));
        return 1;
    }

    set_interface_attribs(ifd1, 921600);

    uint64_t n_written = 0;
    char *buf = malloc(RD_BUFSIZE);
*/
    chameleon_camera_t *camera = open_camera(1, 16, camera_brightness);
    if (camera == NULL) {
        printf("error opening camera\n");
        return 1;
    }

    float shutter, frame_time;
    uint32_t frame_counter;
    char *image_buf = malloc(sizeof(uint16_t) * IMG_WIDTH * IMG_HEIGHT);
    camera_set_framerate(camera, camera_framerate);
    trigger_capture(camera, 0, 1);

    while (1) {
/*
        int n = read(ifd1, buf, RD_BUFSIZE);
        if (n > 0) {
            fwrite(buf, 1, n, f1);
            n_written += n;
        }
        if (n_written > 1024) {
            fflush(f1);
        }
*/
        /* Check for a new image without blocking. */
        if (!capture_wait(camera, &shutter, image_buf, sizeof(uint16_t), 
                (sizeof(uint16_t) * IMG_WIDTH * IMG_HEIGHT), 0,
                &frame_time, &frame_counter)) {
            char image_temp_fname[64];
            char image_fname[64];
            sprintf(image_temp_fname, "img%d.pgm~", frame_counter);
            sprintf(image_fname, "img%d.pgm", frame_counter);
            FILE *image_file = fopen(image_temp_fname, "wb");

            /* Write PGM header information. */
            char pgm_header[64];
            int len = sprintf(pgm_header, "P5\n%d\n%d\n65535\n",
                IMG_WIDTH, IMG_HEIGHT);
            fwrite(pgm_header, sizeof(char), len, image_file);

            /* Write the image to a temporary file. */
            fwrite(image_buf, sizeof(uint16_t), (IMG_WIDTH * IMG_HEIGHT),
                image_file);
            fflush(image_file);
            
            /* Dump all telemetry since the last image to a temporary file. */

            /* Rename the image and telemetry files. */
            rename(image_temp_fname, image_fname);
        }
    }
}

