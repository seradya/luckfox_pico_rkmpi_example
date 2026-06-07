#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <time.h>
#include <unistd.h>
#include <vector>
#include <chrono>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>

#include "rtsp_demo.h"
#include "luckfox_mpi.h"
#include "yolov8.h"
#include "web_server.h"
#include "mqtt_client.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#define DISP_WIDTH  640
#define DISP_HEIGHT 480

#define USE_RTSP_INPUT 1
#define VERBOSE_DETECTIONS 0

// Number of draw buffers in the circular output ring.
// Must be large enough that by the time we wrap around and reuse slot N,
// the encoder has finished reading it (i.e. ReleaseStream for that frame
// has already been called). 3 is the safe minimum for a single-channel VENC:
//   slot 0 — encoder is reading  (SendFrame called, GetStream not yet returned)
//   slot 1 — encoder just finished (GetStream returned, ReleaseStream called)
//   slot 2 — we are painting on this one right now
#define DRAW_BUF_COUNT 5

// HTTP-порт веб-интерфейса настройки камер.
#define WEB_SERVER_PORT 8080

// Минимальный интервал между экспортами (JSON/MQTT) на одну камеру, мс —
// чтобы не заваливать брокер/диск кадрами на полной частоте.
#define EXPORT_MIN_INTERVAL_MS 250

// Глобальный MQTT-публикатор (асинхронный, фоновый поток).
static MqttClient g_mqtt;

int width        = DISP_WIDTH;
int height       = DISP_HEIGHT;
int model_width  = 640;
int model_height = 640;
float scale;
int leftPadding;
int topPadding;

cv::Mat letterbox(const cv::Mat &input)
{
    float scaleX = (float)model_width  / (float)width;
    float scaleY = (float)model_height / (float)height;
    scale = scaleX < scaleY ? scaleX : scaleY;

    int inputWidth  = (int)((float)width  * scale);
    int inputHeight = (int)((float)height * scale);

    leftPadding = (model_width  - inputWidth)  / 2;
    topPadding  = (model_height - inputHeight) / 2;

    // Постоянный буфер: чёрные поля заливаются один раз, далее каждый кадр
    // ресайзим прямо в ROI — без аллокаций cv::Mat и copyTo на каждом кадре.
    static cv::Mat out;
    if (out.empty())
        out = cv::Mat(model_height, model_width, CV_8UC3, cv::Scalar(0, 0, 0));

    cv::resize(input,
               out(cv::Rect(leftPadding, topPadding, inputWidth, inputHeight)),
               cv::Size(inputWidth, inputHeight), 0, 0, cv::INTER_LINEAR);
    return out;
}

// Build the ffmpeg command that decodes one RTSP source to raw rgb24 frames.
// -skip_loop_filter all / -flags2 fast cut software-decode CPU on the weak
// Cortex-A7. -r must match the source fps to avoid speed-up/slow-down.
static std::string build_ffmpeg_cmd(const std::string &url)
{
    return
        "ffmpeg "
        "-rtsp_transport tcp "
        "-fflags nobuffer+discardcorrupt+genpts "
        "-flags low_delay "
        "-flags2 fast "
        "-skip_loop_filter all "
        "-i " + url + " "
        "-map 0:v:0 "
        "-f rawvideo "
        "-pix_fmt rgb24 "
        "-vf scale=" + std::to_string(width) + ":" + std::to_string(height) + " "
        "-an -sn -dn "
        "-loglevel error "
        "-r 15 "
        "-nostdin "
        "-y "
        "-";
}

void mapCoordinates(int *x, int *y)
{
    *x = (int)(((float)(*x - leftPadding)) / scale);
    *y = (int)(((float)(*y - topPadding))  / scale);
}

static double now_seconds()
{
    return std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

// Сериализация результатов инференса одной камеры в JSON (формат в духе
// событий Frigate: камера, метка времени, размер кадра и список детекций
// с боксами в координатах исходного кадра).
static std::string build_detection_json(int cam,
                                        object_detect_result_list *od,
                                        double ts)
{
    char buf[192];
    std::string s = "{";
    snprintf(buf, sizeof(buf),
             "\"camera\":\"cam%d\",\"timestamp\":%.3f,", cam, ts);
    s += buf;
    snprintf(buf, sizeof(buf),
             "\"frame_width\":%d,\"frame_height\":%d,\"detections\":[",
             width, height);
    s += buf;

    for (int i = 0; i < od->count; i++) {
        object_detect_result *d = &od->results[i];
        int x1 = (int)d->box.left,  y1 = (int)d->box.top;
        int x2 = (int)d->box.right, y2 = (int)d->box.bottom;
        mapCoordinates(&x1, &y1);
        mapCoordinates(&x2, &y2);
        if (x1 < 0) x1 = 0;
        if (y1 < 0) y1 = 0;
        if (x2 > width)  x2 = width;
        if (y2 > height) y2 = height;

        snprintf(buf, sizeof(buf),
                 "%s{\"label\":\"%s\",\"score\":%.3f,"
                 "\"box\":[%d,%d,%d,%d],\"area\":%d}",
                 i ? "," : "", coco_cls_to_name(d->cls_id), d->prop,
                 x1, y1, x2, y2, (x2 - x1) * (y2 - y1));
        s += buf;
    }

    s += "]}";
    return s;
}

#if USE_RTSP_INPUT

#define MAX_CAMERAS 4
#define READER_RETRY_SEC 3

static bool read_exact(int fd, unsigned char *buf, size_t frame_size)
{
    size_t done = 0;
    while (done < frame_size) {
        ssize_t r = read(fd, buf + done, frame_size - done);
        if (r > 0) {
            done += (size_t)r;
            continue;
        }
        if (r == 0)
            return false; // EOF
        if (errno == EINTR)
            continue;
        return false;
    }
    return true;
}

// One independent decoder per camera: its own ffmpeg + reader thread, keeping
// only the latest decoded frame. The main loop reads the latest frame from each
// active camera, so switching the output stream between cameras is instant
// (no ffmpeg reconnect) and inference can run on every camera.
struct CameraReader {
    std::string url;
    FILE *pipe = nullptr;
    int   fd   = -1;
    std::thread th;
    std::atomic<bool> running{false};
    bool   started      = false;
    time_t last_attempt = 0;

    std::vector<unsigned char> latest;
    bool frame_ready = false;
    unsigned long long seq           = 0;
    unsigned long long last_consumed = 0;
    std::mutex mtx;
    std::condition_variable cv;
};

static CameraReader g_readers[MAX_CAMERAS];
static size_t       g_frame_size = 0;

static void reader_loop(CameraReader *r)
{
    std::vector<unsigned char> local(g_frame_size);
    while (r->running.load()) {
        if (r->fd < 0 || !read_exact(r->fd, local.data(), g_frame_size))
            break;
        {
            std::lock_guard<std::mutex> lk(r->mtx);
            r->latest.swap(local); // local now owns the previous buffer (same size)
            r->frame_ready = true;
            r->seq++;
        }
        r->cv.notify_all();
    }
    {
        std::lock_guard<std::mutex> lk(r->mtx);
        r->running.store(false);
    }
    r->cv.notify_all();
}

static void stop_reader(CameraReader *r)
{
    if (!r->started) return;
    r->running.store(false);
    if (r->pipe) { pclose(r->pipe); r->pipe = nullptr; r->fd = -1; } // unblocks read()
    if (r->th.joinable()) r->th.join();
    r->started     = false;
    r->frame_ready = false;
    r->url.clear();
}

static bool start_reader(CameraReader *r, const std::string &url)
{
    stop_reader(r);

    std::string cmd = build_ffmpeg_cmd(url);
    printf("Run: %s\n", cmd.c_str());
    r->pipe = popen(cmd.c_str(), "r");
    if (!r->pipe) { printf("ffmpeg popen failed: %s\n", url.c_str()); return false; }
    setvbuf(r->pipe, NULL, _IONBF, 0);
    r->fd = fileno(r->pipe);
    if (r->fd < 0) { pclose(r->pipe); r->pipe = nullptr; return false; }

    r->url           = url;
    r->latest.assign(g_frame_size, 0);
    r->frame_ready   = false;
    r->seq           = 0;
    r->last_consumed = 0;
    r->running.store(true);
    r->th = std::thread(reader_loop, r);
    r->started = true;
    return true;
}

// Copy the latest NEW frame for camera r into buf. Returns false if there is no
// new frame right now (caller should skip this camera this round).
static bool grab_reader_frame(CameraReader *r, unsigned char *buf)
{
    std::lock_guard<std::mutex> lk(r->mtx);
    if (!r->frame_ready || r->seq == r->last_consumed)
        return false;
    memcpy(buf, r->latest.data(), g_frame_size);
    r->last_consumed = r->seq;
    return true;
}

// Reconcile readers with the web-UI camera list: start readers for added/changed
// URLs, stop readers for removed cameras, and respawn dead ones with a backoff.
static void sync_readers()
{
    std::vector<std::string> urls = get_all_cameras();
    time_t now = time(NULL);

    for (int i = 0; i < MAX_CAMERAS && i < (int)urls.size(); i++) {
        CameraReader *r = &g_readers[i];
        const std::string &u = urls[i];

        if (u.empty()) {
            if (r->started) {
                printf("Camera %d removed, stopping reader\n", i);
                stop_reader(r);
            }
            continue;
        }

        bool need = (!r->started) || (r->url != u) || (!r->running.load());
        if (need && (now - r->last_attempt) >= READER_RETRY_SEC) {
            r->last_attempt = now;
            printf("Camera %d (re)starting reader: %s\n", i, u.c_str());
            start_reader(r, u);
        }
    }
}

#endif // USE_RTSP_INPUT

int main(int argc, char *argv[])
{
    system("RkLunch-stop.sh");

    RK_S32 s32Ret = 0;
    int sX, sY, eX, eY;
    char text[64];

    // -------------------------------------------------------------------------
    // Camera configuration + web server
    //
    // На старте НИ ОДИН стрим не запускается: камеры задаются через веб-интерфейс
    // (порт WEB_SERVER_PORT). Только если URL передан явно в argv[1], он
    // подставляется в слот 0 (для отладки одним источником).
    // -------------------------------------------------------------------------
    if (argc > 1 && argv[1][0] != '\0')
    {
        std::lock_guard<std::mutex> lk(cameras_mutex);
        cameras[0] = argv[1];
    }
    std::thread(run_web_server, WEB_SERVER_PORT).detach();
    printf("Web UI: http://<board-ip>:%d (configure up to 4 cameras)\n",
           WEB_SERVER_PORT);

    g_mqtt.start();   // фоновый MQTT-публикатор (бездействует, пока не настроен)

    // -------------------------------------------------------------------------
    // RKNN model init
    // -------------------------------------------------------------------------
    rknn_app_context_t rknn_app_ctx;
    object_detect_result_list od_results;
    memset(&rknn_app_ctx, 0, sizeof(rknn_app_context_t));
    init_yolov8_model("./model/yolov8.rknn", &rknn_app_ctx);
    printf("init rknn model success!\n");
    init_post_process();

    // -------------------------------------------------------------------------
    // VENC stream handle
    // -------------------------------------------------------------------------
    VENC_STREAM_S stFrame;
    stFrame.pstPack = (VENC_PACK_S *)malloc(sizeof(VENC_PACK_S));
    RK_U32 H264_TimeRef = 0;
    VIDEO_FRAME_INFO_S stViFrame;

    const size_t frame_bytes = (size_t)width * height * 3;
#if USE_RTSP_INPUT
    g_frame_size = frame_bytes;
#endif

    // -------------------------------------------------------------------------
    // DMA pool — 1 source block + DRAW_BUF_COUNT draw blocks
    //
    //  src_Blk          — raw pixels land here (fread / VI); inference reads it;
    //                     the encoder NEVER touches this block.
    //
    //  draw_Blk[N]      — circular ring of annotated output frames.
    //                     Each iteration we advance draw_idx, copy src → current
    //                     slot, paint overlays, then hand it to VENC.  By the
    //                     time we wrap around and reuse a slot, ReleaseStream
    //                     has already been called for it, so the encoder is done.
    // -------------------------------------------------------------------------
    MB_POOL_CONFIG_S PoolCfg;
    memset(&PoolCfg, 0, sizeof(MB_POOL_CONFIG_S));
    PoolCfg.u64MBSize   = (RK_U64)frame_bytes;
    PoolCfg.u32MBCnt    = 1 + DRAW_BUF_COUNT;
    PoolCfg.enAllocType = MB_ALLOC_TYPE_DMA;
    MB_POOL src_Pool = RK_MPI_MB_CreatePool(&PoolCfg);
    printf("Create Pool success!\n");

    // Source block
    MB_BLK src_Blk  = RK_MPI_MB_GetMB(src_Pool, (RK_U64)frame_bytes, RK_TRUE);
    unsigned char *src_data = (unsigned char *)RK_MPI_MB_Handle2VirAddr(src_Blk);
    cv::Mat src_frame(height, width, CV_8UC3, src_data);

    // Draw ring
    MB_BLK        draw_Blk[DRAW_BUF_COUNT];
    unsigned char *draw_data[DRAW_BUF_COUNT];
    cv::Mat        draw_frame[DRAW_BUF_COUNT];

    for (int i = 0; i < DRAW_BUF_COUNT; i++) {
        draw_Blk[i]  = RK_MPI_MB_GetMB(src_Pool, (RK_U64)frame_bytes, RK_TRUE);
        draw_data[i] = (unsigned char *)RK_MPI_MB_Handle2VirAddr(draw_Blk[i]);
        draw_frame[i] = cv::Mat(height, width, CV_8UC3, draw_data[i]);
    }

    int draw_idx = 0; // next slot to write into

    // h264_frame is a template; we update pMbBlk each iteration
    VIDEO_FRAME_INFO_S h264_frame;
    h264_frame.stVFrame.u32Width      = width;
    h264_frame.stVFrame.u32Height     = height;
    h264_frame.stVFrame.u32VirWidth   = width;
    h264_frame.stVFrame.u32VirHeight  = height;
    h264_frame.stVFrame.enPixelFormat = RK_FMT_RGB888;
    h264_frame.stVFrame.u32FrameFlag  = 160;
    h264_frame.stVFrame.pMbBlk        = draw_Blk[0]; // updated each loop

#if !USE_RTSP_INPUT
    RK_BOOL multi_sensor = RK_FALSE;
    const char *iq_dir = "/etc/iqfiles";
    rk_aiq_working_mode_t hdr_mode = RK_AIQ_WORKING_MODE_NORMAL;
    SAMPLE_COMM_ISP_Init(0, hdr_mode, multi_sensor, iq_dir);
    SAMPLE_COMM_ISP_Run(0);
#endif

    if (RK_MPI_SYS_Init() != RK_SUCCESS) {
        RK_LOGE("rk mpi sys init fail!");
        return -1;
    }

    // RTSP output server
    rtsp_demo_handle    g_rtsplive    = create_rtsp_demo(777);
    rtsp_session_handle g_rtsp_session = rtsp_new_session(g_rtsplive, "/live/0");
    rtsp_set_video(g_rtsp_session, RTSP_CODEC_ID_VIDEO_H264, NULL, 0);
    rtsp_sync_video_ts(g_rtsp_session, rtsp_get_reltime(), rtsp_get_ntptime());

#if !USE_RTSP_INPUT
    vi_dev_init();
    vi_chn_init(0, width, height);
#endif

    venc_init(0, width, height, RK_VIDEO_ID_AVC);
    printf("venc init success\n");

    float fps      = 0.0f;
    char  fps_text[32];

#if USE_RTSP_INPUT
    while (1)
    {
        // ------------------------------------------------------------------
        // Step 1: keep one ffmpeg decoder per configured camera in sync with
        //         the web UI, then round-robin across the active cameras so
        //         inference runs for ALL of them. Only the camera selected via
        //         the web "Stream" button is encoded to the RTSP output.
        // ------------------------------------------------------------------
        sync_readers();

        int active[MAX_CAMERAS];
        int nactive = 0;
        for (int i = 0; i < MAX_CAMERAS; i++)
            if (g_readers[i].started)
                active[nactive++] = i;

        if (nactive == 0)
        {
            printf("No camera configured. Open web UI at :%d to add one. Waiting...\n",
                   WEB_SERVER_PORT);
            sleep(2);
            continue;
        }

        static int rr = 0;
        int ci  = active[rr % nactive];
        int sel = g_stream_camera.load();
        rr++;

        if (!grab_reader_frame(&g_readers[ci], src_data))
        {
            usleep(5000); // no new frame from this camera yet — avoid busy spin
            continue;
        }
#else
    while (1)
    {
        s32Ret = RK_MPI_VI_GetChnFrame(0, 0, &stViFrame, -1);
        if (s32Ret == RK_SUCCESS)
        {
            void *vi_data = RK_MPI_MB_Handle2VirAddr(stViFrame.stVFrame.pMbBlk);
            cv::Mat yuv420sp(height + height / 2, width, CV_8UC1, vi_data);
            cv::cvtColor(yuv420sp, src_frame, cv::COLOR_YUV420sp2RGB);
        }
        int ci = 0, sel = 0; (void)ci; (void)sel;
#endif

        // ------------------------------------------------------------------
        // Step 2: inference on src_frame (clean, no overlays ever painted here)
        // ------------------------------------------------------------------
        cv::Mat letterboxImage = letterbox(src_frame);

		// static int frame_id = 0;
		// char filename[256];
		// snprintf(filename,
		// 		sizeof(filename),
		// 		"/tmp/frame_%06d.jpg",
		// 		frame_id++);
					
		// cv::imwrite(filename, src_frame);

        memcpy(rknn_app_ctx.input_mems[0]->virt_addr,
               letterboxImage.data, model_width * model_height * 3);

        auto infer_start = std::chrono::high_resolution_clock::now();
        int ret = rknn_run(rknn_app_ctx.rknn_ctx, nullptr);
        auto infer_end   = std::chrono::high_resolution_clock::now();

        if (ret < 0) {
            printf("RKNN run failed! Error code: %d\n", ret);
            continue;
        }

        fps = 1000.0f / std::chrono::duration<float, std::milli>(
                            infer_end - infer_start).count();

        memset(&od_results, 0, sizeof(od_results));
        post_process(&rknn_app_ctx, rknn_app_ctx.output_mems,
                     0.25f, 0.45f, &od_results);

#if USE_RTSP_INPUT
        // Лёгкий статус-лог не чаще раза в 3 с на камеру (печать в консоль на
        // каждом кадре заметно роняет FPS, особенно через adb/serial).
        {
            static double last_log[MAX_CAMERAS] = {0};
            double tnow = now_seconds();
            if (tnow - last_log[ci] >= 3.0) {
                last_log[ci] = tnow;
                printf("[cam %d] %d obj, %.1f infer-fps%s\n",
                       ci, od_results.count, fps, (ci == sel) ? " [STREAM]" : "");
            }
        }

        // ------------------------------------------------------------------
        // Экспорт результатов инференса: JSON-файл (для Frigate/NVR) и/или
        // публикация по MQTT. Делается для КАЖДОЙ камеры, с троттлингом.
        // ------------------------------------------------------------------
        {
            static double last_export[MAX_CAMERAS] = {0};
            MqttSettings ms = get_mqtt_settings();

            if (ms.mqtt_enabled || ms.json_enabled)
            {
                double now_s = now_seconds();
                if ((now_s - last_export[ci]) * 1000.0 >= EXPORT_MIN_INTERVAL_MS)
                {
                    last_export[ci] = now_s;
                    std::string js = build_detection_json(ci, &od_results, now_s);

                    if (ms.json_enabled) {
                        std::string path =
                            ms.json_dir + "/cam" + std::to_string(ci) + ".json";
                        FILE *f = fopen(path.c_str(), "w");
                        if (f) { fwrite(js.data(), 1, js.size(), f); fclose(f); }
                    }

                    if (ms.mqtt_enabled && !ms.host.empty()) {
                        g_mqtt.configure(ms.host, ms.port, ms.user, ms.pass);
                        g_mqtt.publish(ms.base_topic + "/cam" + std::to_string(ci), js);
                    } else {
                        g_mqtt.configure("", 0, "", ""); // выключен — отключаемся
                    }
                }
            }
        }
#endif

        // Only the camera selected via the web "Stream" button is annotated,
        // encoded and pushed to the RTSP output. Other cameras run inference
        // (logged above) but produce no output.
        if (ci == sel)
        {
        h264_frame.stVFrame.u32TimeRef = H264_TimeRef++;
        h264_frame.stVFrame.u64PTS     = TEST_COMM_GetNowUs();

        // ------------------------------------------------------------------
        // Step 3: pick next draw buffer from the ring, copy src into it,
        //         then paint overlays — encoder never sees this slot again
        //         until DRAW_BUF_COUNT iterations later (after ReleaseStream)
        // ------------------------------------------------------------------
        int cur = draw_idx;
        draw_idx = (draw_idx + 1) % DRAW_BUF_COUNT;

        memcpy(draw_data[cur], src_data, frame_bytes);

        for (int i = 0; i < od_results.count; i++)
        {
            object_detect_result *det = &od_results.results[i];

            sX = (int)det->box.left;
            sY = (int)det->box.top;
            eX = (int)det->box.right;
            eY = (int)det->box.bottom;
            mapCoordinates(&sX, &sY);
            mapCoordinates(&eX, &eY);

#if VERBOSE_DETECTIONS
            printf("%s @ (%d %d %d %d) %.3f\n",
                   coco_cls_to_name(det->cls_id),
                   sX, sY, eX, eY, det->prop);
#endif

            cv::rectangle(draw_frame[cur],
                          cv::Point(sX, sY), cv::Point(eX, eY),
                          cv::Scalar(0, 255, 0), 3);

            snprintf(text, sizeof(text), "%s %.1f%%",
                     coco_cls_to_name(det->cls_id), det->prop * 100);
            cv::putText(draw_frame[cur], text, cv::Point(sX, sY - 8),
                        cv::FONT_HERSHEY_SIMPLEX, 1,
                        cv::Scalar(0, 255, 0), 2);
        }

        // Реальный FPS выходного видео: интервал между отданными кадрами
        // ВЫБРАННОЙ камеры (а не время инференса). При нескольких камерах на
        // выбранную приходится меньше кадров → частота честно падает.
        // Сглаживаем экспоненциальным средним, чтобы цифра не дёргалась.
        static auto  last_out_ts  = std::chrono::steady_clock::now();
        static float out_fps      = 0.0f;
        static int   last_out_cam = -1;
        auto  now_ts = std::chrono::steady_clock::now();
        float dt_ms  = std::chrono::duration<float, std::milli>(now_ts - last_out_ts).count();
        if (ci == last_out_cam && dt_ms > 0.0f) {
            float inst = 1000.0f / dt_ms;
            out_fps = (out_fps <= 0.0f) ? inst : (out_fps * 0.9f + inst * 0.1f);
        }
        last_out_ts  = now_ts;
        last_out_cam = ci;

        snprintf(fps_text, sizeof(fps_text), "FPS: %.2f", out_fps);
        cv::putText(draw_frame[cur], fps_text, cv::Point(20, 40),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0,
                    cv::Scalar(0, 0, 255), 2);

#if USE_RTSP_INPUT
        char cam_text[32];
        snprintf(cam_text, sizeof(cam_text), "CAM %d", ci + 1);
        cv::putText(draw_frame[cur], cam_text, cv::Point(20, 75),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0,
                    cv::Scalar(255, 255, 0), 2);
#endif

		// static int frame_id = 0;
		// char filename[256];
		// snprintf(filename,
		// 		sizeof(filename),
		// 		"/tmp/frame_%06d.jpg",
		// 		frame_id++);
					
		// cv::imwrite(filename, draw_frame[cur]);

		RK_MPI_SYS_MmzFlushCache(draw_Blk[cur], RK_TRUE);

        // ------------------------------------------------------------------
        // Step 4: hand draw_Blk[cur] to encoder; it stays alive until
        //         ReleaseStream — by then draw_idx has moved on
        // ------------------------------------------------------------------
        h264_frame.stVFrame.pMbBlk = draw_Blk[cur];
        RK_MPI_VENC_SendFrame(0, &h264_frame, -1);

        bool got_stream = false;
        s32Ret = RK_MPI_VENC_GetStream(0, &stFrame, -1);
        if (s32Ret == RK_SUCCESS)
        {
            got_stream = true;
            if (g_rtsplive && g_rtsp_session)
            {
                void *pData = RK_MPI_MB_Handle2VirAddr(stFrame.pstPack->pMbBlk);
                rtsp_tx_video(g_rtsp_session,
                              (uint8_t *)pData,
                              stFrame.pstPack->u32Len,
                              stFrame.pstPack->u64PTS);
                rtsp_do_event(g_rtsplive);
            }
        }

#if !USE_RTSP_INPUT
        s32Ret = RK_MPI_VI_ReleaseChnFrame(0, 0, &stViFrame);
        if (s32Ret != RK_SUCCESS)
            RK_LOGE("RK_MPI_VI_ReleaseChnFrame fail %x", s32Ret);
#endif

        // After ReleaseStream the encoder is guaranteed done with draw_Blk[cur]
        if (got_stream) {
            s32Ret = RK_MPI_VENC_ReleaseStream(0, &stFrame);
            if (s32Ret != RK_SUCCESS)
                RK_LOGE("RK_MPI_VENC_ReleaseStream fail %x", s32Ret);
        }

        } // end if (ci == sel)

        memset(text, 0, sizeof(text));
    }

    // -------------------------------------------------------------------------
    // Cleanup
    // -------------------------------------------------------------------------
    RK_MPI_MB_ReleaseMB(src_Blk);
    for (int i = 0; i < DRAW_BUF_COUNT; i++)
        RK_MPI_MB_ReleaseMB(draw_Blk[i]);
    RK_MPI_MB_DestroyPool(src_Pool);

#if !USE_RTSP_INPUT
    RK_MPI_VI_DisableChn(0, 0);
    RK_MPI_VI_DisableDev(0);
    SAMPLE_COMM_ISP_Stop(0);
#endif

    RK_MPI_VENC_StopRecvFrame(0);
    RK_MPI_VENC_DestroyChn(0);
    free(stFrame.pstPack);

    if (g_rtsplive) rtsp_del_demo(g_rtsplive);

#if USE_RTSP_INPUT
    for (int i = 0; i < MAX_CAMERAS; i++)
        stop_reader(&g_readers[i]);
#endif

    RK_MPI_SYS_Exit();
    release_yolov8_model(&rknn_app_ctx);
    deinit_post_process();
    return 0;
}