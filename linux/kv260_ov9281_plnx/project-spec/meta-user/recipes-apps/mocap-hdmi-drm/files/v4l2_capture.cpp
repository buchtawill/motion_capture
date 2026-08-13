#include "v4l2_capture.hpp"

#include <cerrno>
#include <cstring>
#include <iostream>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/dma-buf.h>
#include <linux/videodev2.h>

#include <mocap/ov9281_pipeline.hpp>

using namespace mocap;

void dmabuf_sync_read(int fd, bool start) {
    dma_buf_sync s{};
    s.flags = (start ? DMA_BUF_SYNC_START : DMA_BUF_SYNC_END) | DMA_BUF_SYNC_READ;
    ioctl(fd, DMA_BUF_IOCTL_SYNC, &s);
}

V4l2Capture v4l2_open_capture(const std::string &device, unsigned width,
                               unsigned height, uint32_t pixfmt,
                               uint32_t luma_stride, unsigned disp_w,
                               unsigned disp_h) {
    V4l2Capture cap;
    cap.fd = open(device.c_str(), O_RDWR, 0);
    if (cap.fd == -1)
        fail("open " + device);

    v4l2_capability vcap{};
    if (xioctl(cap.fd, VIDIOC_QUERYCAP, &vcap) == -1)
        fail("VIDIOC_QUERYCAP");
    cap.mplane = vcap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE;
    cap.buf_type = cap.mplane ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
                              : V4L2_BUF_TYPE_VIDEO_CAPTURE;

    // Force the capture stride to the 1920-wide luma pitch so the DMA writes
    // each sensor line into the top-left of the 1080p buffer (the interleaved
    // DMA's inter-line gap = stride - line bytes leaves the right side black).
    v4l2_format fmt{};
    fmt.type = cap.buf_type;
    if (cap.mplane) {
        fmt.fmt.pix_mp.width = width;
        fmt.fmt.pix_mp.height = height;
        fmt.fmt.pix_mp.pixelformat = pixfmt;
        fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
        fmt.fmt.pix_mp.num_planes = 1;
        fmt.fmt.pix_mp.plane_fmt[0].bytesperline = luma_stride;
        fmt.fmt.pix_mp.plane_fmt[0].sizeimage = luma_stride * height;
    } else {
        fmt.fmt.pix.width = width;
        fmt.fmt.pix.height = height;
        fmt.fmt.pix.pixelformat = pixfmt;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
        fmt.fmt.pix.bytesperline = luma_stride;
        fmt.fmt.pix.sizeimage = luma_stride * height;
    }
    if (xioctl(cap.fd, VIDIOC_S_FMT, &fmt) == -1)
        fail("VIDIOC_S_FMT");

    cap.n_planes = cap.mplane ? fmt.fmt.pix_mp.num_planes : 1;
    cap.width = cap.mplane ? fmt.fmt.pix_mp.width : fmt.fmt.pix.width;
    cap.height = cap.mplane ? fmt.fmt.pix_mp.height : fmt.fmt.pix.height;
    cap.bytesperline = cap.mplane ? fmt.fmt.pix_mp.plane_fmt[0].bytesperline
                                  : fmt.fmt.pix.bytesperline;
    cap.pixfmt = cap.mplane ? fmt.fmt.pix_mp.pixelformat : fmt.fmt.pix.pixelformat;

    if (cap.pixfmt != V4L2_PIX_FMT_GREY)
        die("driver did not accept GREY (got '" + fourcc_str(cap.pixfmt) + "')");
    if (cap.width > disp_w || cap.height > disp_h)
        die("capture " + std::to_string(cap.width) + "x" +
            std::to_string(cap.height) + " exceeds the " +
            std::to_string(disp_w) + "x" + std::to_string(disp_h) +
            " scanout buffer");
    // The DMA stride must match the luma buffer pitch exactly or the image
    // shears; the driver may round bytesperline to its own alignment.
    if (cap.bytesperline != luma_stride)
        die("driver forced stride " + std::to_string(cap.bytesperline) +
            " != luma pitch " + std::to_string(luma_stride) +
            " (stride-alignment mismatch)");

    std::cout << "Capture: " << cap.width << "x" << cap.height << " '"
              << fourcc_str(cap.pixfmt) << "' into " << disp_w << "x" << disp_h
              << " top-left, stride " << cap.bytesperline
              << (cap.mplane ? " [mplane]" : "") << "\n";

    return cap;
}

void v4l2_setup_dmabuf_buffers(V4l2Capture &cap, std::vector<LumaSlot> &slot,
                                unsigned nbuf) {
    v4l2_requestbuffers req{};
    req.count = nbuf;
    req.type = cap.buf_type;
    req.memory = V4L2_MEMORY_DMABUF;
    if (xioctl(cap.fd, VIDIOC_REQBUFS, &req) == -1)
        fail("VIDIOC_REQBUFS (DMABUF import; does this driver support it?)");
    if (req.count < nbuf)
        die("driver gave " + std::to_string(req.count) +
            " DMABUF slots, fewer than the " + std::to_string(nbuf) +
            " luma buffers");

    for (unsigned i = 0; i < nbuf; ++i)
        v4l2_qbuf(cap, slot, static_cast<int>(i));
}

void v4l2_qbuf(const V4l2Capture &cap, const std::vector<LumaSlot> &slot,
               int i) {
    v4l2_buffer b{};
    v4l2_plane pl[VIDEO_MAX_PLANES]{};
    b.type = cap.buf_type;
    b.memory = V4L2_MEMORY_DMABUF;
    b.index = static_cast<uint32_t>(i);
    if (cap.mplane) {
        b.length = cap.n_planes;
        b.m.planes = pl;
        pl[0].m.fd = slot[i].dmabuf_fd;
        pl[0].length = slot[i].size;
    } else {
        b.m.fd = slot[i].dmabuf_fd;
        b.length = slot[i].size;
    }
    if (xioctl(cap.fd, VIDIOC_QBUF, &b) == -1)
        fail("VIDIOC_QBUF (DMABUF)");
}
