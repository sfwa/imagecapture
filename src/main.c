#include <errno.h>
#include <asm/termios.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdint.h>
#include <poll.h>
#include "chameleon_util.h"

#define RD_BUFSIZE 32768u

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

    if (argc < 2) {
        printf("Usage: fcsdump /PATH/TO/DIR");
        return 1;
    }

    char template[128];
    snprintf(template, 128, "%s/images-XXXXXX", argv[1]);
    char *dname = mkdtemp(template);

    if (!dname) {
        printf("error %d creating random directory: %s\n",
            errno, strerror(errno));
        return 1;
    }

    int ifd1 = open("/dev/ttySAC0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (ifd1 < 0) {
        printf("error %d opening /dev/ttySAC0: %s\n", errno, strerror(errno));
        return 1;
    }

    int flags = fcntl(ifd1, F_GETFL, 0);
    if(fcntl(ifd1, F_SETFL, flags | O_NONBLOCK)) {
        printf("error %d setting socket to non-blocking mode: %s\n", errno,
            strerror(errno));
        return 1;
    }

    set_interface_attribs(ifd1, 115200);

    uint32_t tail = 0;
    char *buf = malloc(RD_BUFSIZE);

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
        /* Check for a new image, 10ms timeout. */
        int ret = capture_wait(camera, &shutter, image_buf,
            sizeof(uint16_t), (sizeof(uint16_t) * IMG_WIDTH * IMG_HEIGHT),
            10, &frame_time, &frame_counter);

        /* Non-blocking telemetry read. */
        int n = read(ifd1, &buf[tail], RD_BUFSIZE);
        if (n > 0) {
            tail += n;
        }

        if (!ret) {
            char image_temp_fname[128];
            char image_fname[128];
            snprintf(image_temp_fname, 128, "%s/img%d.pgm~", dname,
                frame_counter);
            snprintf(image_fname, 128, "%s/img%d.pgm", dname,
                frame_counter);
            FILE *image_file = fopen(image_temp_fname, "wb");

            /* Write PGM header information. */
            char pgm_header[64];
            int len = snprintf(pgm_header, 64, "P5\n%d\n%d\n65535\n",
                IMG_WIDTH, IMG_HEIGHT);
            fwrite(pgm_header, sizeof(char), len, image_file);

            /* Write the image to a temporary file. */
            fwrite(image_buf, sizeof(uint16_t), (IMG_WIDTH * IMG_HEIGHT),
                image_file);
            fflush(image_file);
            fclose(image_file);

            /* Dump all telemetry since last image to a temporary file. */
            char telemetry_temp_fname[128];
            char telemetry_fname[128];
            snprintf(telemetry_temp_fname, 128, "%s/telem%d.txt~", dname,
                frame_counter);
            snprintf(telemetry_fname, 128, "%s/telem%d.txt", dname,
                frame_counter);
            FILE *telemetry_file = fopen(telemetry_temp_fname, "wb");

            /* Find the last packet boundary. */
            uint32_t buf_len = 0;
            int i;
            for (i = tail-2; i >= 0; i--) {
                if (buf[i] == 0x00 && buf[i+1] == 0x00) {
                    buf_len = i+2;
                    break;
                }
            }
            fwrite(buf, sizeof(char), buf_len, telemetry_file);
            fflush(telemetry_file);
            fclose(telemetry_file);

            /* Relocate the remainder to the start of the buffer. */
            memcpy(buf, &buf[i+2], tail-buf_len);
            tail -= buf_len;

            /* Rename the image and telemetry files. */
            rename(telemetry_temp_fname, telemetry_fname);
            rename(image_temp_fname, image_fname);
        }
    }
}

