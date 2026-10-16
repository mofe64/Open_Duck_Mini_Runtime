#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <linux/media-bus-format.h>
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

static int (*next_ioctl)(int, unsigned long, ...);
static pthread_once_t resolve_once = PTHREAD_ONCE_INIT;

static void resolve_ioctl(void) {
    *(void **)(&next_ioctl) = dlsym(RTLD_NEXT, "ioctl");
}

static int is_target_sensor(int fd) {
    char link[64], target[128];
    snprintf(link, sizeof(link), "/proc/self/fd/%d", fd);
    ssize_t n = readlink(link, target, sizeof(target) - 1);
    if (n < 0) return 0;
    target[n] = '\0';
    return strcmp(target, "/dev/v4l-subdev0") == 0;
}

int ioctl(int fd, unsigned long request, ...) {
    va_list args;
    va_start(args, request);
    uintptr_t argument = va_arg(args, uintptr_t);
    va_end(args);
    pthread_once(&resolve_once, resolve_ioctl);
    if (!next_ioctl) { errno = ENOSYS; return -1; }

    int result = next_ioctl(fd, request, argument);
    int saved_errno = errno;
    if (result != -1 || saved_errno != EPERM ||
        (unsigned int)request != VIDIOC_SUBDEV_S_FMT || !argument || !is_target_sensor(fd)) {
        errno = saved_errno;
        return result;
    }

    struct v4l2_subdev_format *requested = (void *)argument;
    if (requested->which != V4L2_SUBDEV_FORMAT_ACTIVE ||
        requested->pad != 0) {
        errno = saved_errno;
        return result;
    }

    struct v4l2_subdev_capability caps = {0};
    struct v4l2_subdev_format actual = {
        .which = requested->which, .pad = requested->pad, .stream = 0
    };
    if (next_ioctl(fd, VIDIOC_SUBDEV_QUERYCAP, &caps) != 0 ||
        !(caps.capabilities & V4L2_SUBDEV_CAP_RO_SUBDEV) ||
        (caps.capabilities & V4L2_SUBDEV_CAP_STREAMS) ||
        next_ioctl(fd, VIDIOC_SUBDEV_G_FMT, &actual) != 0) {
        errno = saved_errno;
        return result;
    }

    const struct v4l2_mbus_framefmt *want = &requested->format;
    const struct v4l2_mbus_framefmt *have = &actual.format;
    fprintf(stderr, "Camera format: requested=%ux%u code=0x%x field=%u; active=%ux%u code=0x%x field=%u\n",
            want->width, want->height, want->code, want->field,
            have->width, have->height, have->code, have->field);
    int matching = want->width == 240 && want->height == 180 &&
        want->code == MEDIA_BUS_FMT_Y12_1X12 &&
        have->width == want->width && have->height == want->height &&
        have->code == want->code &&
        (want->field == V4L2_FIELD_ANY || want->field == have->field);
    if (!matching) {
        fprintf(stderr, "Camera formats differ. Keep the format error.\n");
        errno = saved_errno;
        return result;
    }

    /* The video device already selected this format. Keep the sensor state.
       Return the active format only after all checks pass. */
    *requested = actual;
    fprintf(stderr, "Camera format checked. Keep the existing sensor format.\n");
    errno = 0;
    return 0;
}
