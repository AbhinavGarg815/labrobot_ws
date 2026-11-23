// Measure D435i link health straight from librealsense, with no ROS in the path.

#include <librealsense2/rs.hpp>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace {

struct StreamStats {
  long long received = 0;
  long long first = -1;
  long long last = -1;

  void add(long long num) {
    if (first < 0) first = num;
    last = num;
    ++received;
  }
  // Frame numbers are contiguous, so the count the camera believes it sent is simply the span.
  long long expected() const {
    return (first < 0 || last < first) ? 0 : (last - first + 1);
  }
  long long dropped() const { return expected() - received; }
  double drop_pct() const {
    const long long e = expected();
    return e > 0 ? 100.0 * double(dropped()) / double(e) : 0.0;
  }
};

}  // namespace

int main(int argc, char** argv) {
  const double seconds = (argc > 1) ? std::atof(argv[1]) : 30.0;
  const bool with_color = (argc > 2) ? (std::string(argv[2]) == "1") : true;
  const int fps = (argc > 3) ? std::atoi(argv[3]) : 30;

  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, fps);
  if (with_color) {
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, fps);
  }

  rs2::pipeline pipe;
  rs2::pipeline_profile prof;
  try {
    prof = pipe.start(cfg);
  } catch (const rs2::error& e) {
    std::fprintf(stderr, "pipeline start failed: %s\n", e.what());
    std::fprintf(stderr, "(is the realsense2_camera node still holding the device?)\n");
    return 2;
  }

  auto dev = prof.get_device();
  const char* usb_type = dev.supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR)
                             ? dev.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR)
                             : "unknown";
  std::printf("  device   : %s (fw %s)\n",
              dev.get_info(RS2_CAMERA_INFO_NAME),
              dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION));
  std::printf("  usb type : %s\n", usb_type);
  std::printf("  streaming %.0f s at 848x480x%d depth%s (%d fps)...\n\n",
              seconds, fps, with_color ? " + 640x480 colour" : "", fps);

  const double kWarmup = 2.0;   // auto-exposure settling is not a link fault
  const double kNominalMs = 1000.0 / double(fps);

  const auto t0 = std::chrono::steady_clock::now();
  StreamStats depth, color;
  double prev_ts = 0.0, worst_gap = 0.0;
  int stalls = 0, timeouts = 0;

  while (true) {
    const double el =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    if (el >= seconds) break;

    rs2::frameset fs;
    if (!pipe.try_wait_for_frames(&fs, 2000)) {
      ++timeouts;
      std::printf("  [%6.1fs] *** no frameset within 2 s ***\n", el);
      continue;
    }
    if (el < kWarmup) continue;

    if (auto d = fs.get_depth_frame()) {
      const double ts = d.get_timestamp();  // ms, camera clock
      if (prev_ts > 0.0) {
        const double gap = ts - prev_ts;
        if (gap > worst_gap) worst_gap = gap;
        if (gap > 5.0 * kNominalMs) ++stalls;
      }
      prev_ts = ts;
      depth.add(static_cast<long long>(d.get_frame_number()));
    }
    if (with_color) {
      if (auto c = fs.get_color_frame()) {
        color.add(static_cast<long long>(c.get_frame_number()));
      }
    }
  }
  pipe.stop();

  const double el =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() - kWarmup;

  std::printf("  depth  : %lld frames  %.2f fps   dropped %lld (%.2f%%)\n",
              depth.received, depth.received / el, depth.dropped(), depth.drop_pct());
  if (with_color) {
    std::printf("  colour : %lld frames  %.2f fps   dropped %lld (%.2f%%)\n",
                color.received, color.received / el, color.dropped(), color.drop_pct());
  }
  std::printf("  worst capture gap %.1f ms   stalls %d   frameset timeouts %d\n",
              worst_gap, stalls, timeouts);

  const bool ok = depth.drop_pct() < 0.5 && (!with_color || color.drop_pct() < 0.5) &&
                  stalls == 0 && timeouts == 0 && depth.received > 10;
  std::printf("  LINK: %s\n", ok ? "clean" : "DEGRADED");
  return ok ? 0 : 1;
}
