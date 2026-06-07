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

#include "rtsp_demo.h"
#include "luckfox_mpi.h"
#include "yolov8.h"

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

const std::string RTSP_URL = "rtsp://172.32.0.100:8554/live";

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

    cv::Mat inputScale;
    cv::resize(input, inputScale,
               cv::Size(inputWidth, inputHeight), 0, 0, cv::INTER_LINEAR);
    cv::Mat letterboxImage(model_height, model_width, CV_8UC3, cv::Scalar(0, 0, 0));
    inputScale.copyTo(letterboxImage(cv::Rect(leftPadding, topPadding,
                                              inputWidth, inputHeight)));
    return letterboxImage;
}

FILE *ffmpeg_pipe = nullptr;
int ffmpeg_fd = -1;
pthread_t rtsp_reader_tid;
bool rtsp_reader_started = false;
bool rtsp_reader_running = false;
size_t rtsp_frame_size = 0;
std::vector<unsigned char> rtsp_latest_frame;
bool rtsp_frame_ready = false;
unsigned long long rtsp_frame_seq = 0;
unsigned long long rtsp_last_consumed_seq = 0;
pthread_mutex_t rtsp_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t rtsp_cond = PTHREAD_COND_INITIALIZER;

bool open_rtsp_pipe()
{
    // -skip_loop_filter all / -flags2 fast : cut software-decode CPU on the weak
    // Cortex-A7 so ffmpeg can keep up with the source. If the board decodes too
    // slowly, mediamtx drops frames upstream, breaking the H264 GOP and producing
    // the "corrupted macroblock" garbage seen on screen.
    std::string cmd =
        "ffmpeg "
        "-rtsp_transport tcp "
        "-fflags nobuffer+discardcorrupt+genpts "
        "-flags low_delay "
        "-flags2 fast "
        "-skip_loop_filter all "
        "-i " + RTSP_URL + " "
        "-map 0:v:0 "
        "-f rawvideo "
        "-pix_fmt rgb24 "
        "-vf scale=" + std::to_string(width) + ":" + std::to_string(height) + " "
        "-an -sn -dn "
        "-loglevel error "
        "-r 15 "  // Должно совпадать с fps источника, иначе видео ускоряется/замедляется
        "-nostdin "  // Не ждать ввода с stdin
        "-y "  // Перезаписывать выход
        "-";

    printf("Run: %s\n", cmd.c_str());
    ffmpeg_pipe = popen(cmd.c_str(), "r");
    if (!ffmpeg_pipe) {
        printf("Failed to open ffmpeg pipe\n");
        return false;
    }
    setvbuf(ffmpeg_pipe, NULL, _IONBF, 0);
    ffmpeg_fd = fileno(ffmpeg_pipe);
    if (ffmpeg_fd < 0) {
        printf("Failed to get ffmpeg pipe fd\n");
        pclose(ffmpeg_pipe);
        ffmpeg_pipe = nullptr;
        return false;
    }
    return true;
}

static void stop_rtsp_reader()
{
    if (!rtsp_reader_started)
        return;

    rtsp_reader_running = false;
    if (ffmpeg_pipe) {
        pclose(ffmpeg_pipe); // unblocks read() in reader thread
        ffmpeg_pipe = nullptr;
        ffmpeg_fd = -1;
    }
    pthread_cond_broadcast(&rtsp_cond);
    pthread_join(rtsp_reader_tid, nullptr);
    rtsp_reader_started = false;
}

void mapCoordinates(int *x, int *y)
{
    *x = (int)(((float)(*x - leftPadding)) / scale);
    *y = (int)(((float)(*y - topPadding))  / scale);
}

#if USE_RTSP_INPUT
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

static void *rtsp_reader_thread(void *arg)
{
    (void)arg;
    std::vector<unsigned char> local(rtsp_frame_size);

    while (rtsp_reader_running) {
        if (ffmpeg_fd < 0 || !read_exact(ffmpeg_fd, local.data(), rtsp_frame_size))
            break;

        pthread_mutex_lock(&rtsp_mutex);
        rtsp_latest_frame.swap(local);
        rtsp_frame_ready = true;
        rtsp_frame_seq++;
        pthread_cond_signal(&rtsp_cond);
        pthread_mutex_unlock(&rtsp_mutex);
    }

    pthread_mutex_lock(&rtsp_mutex);
    rtsp_reader_running = false;
    pthread_cond_broadcast(&rtsp_cond);
    pthread_mutex_unlock(&rtsp_mutex);
    return nullptr;
}

static bool start_rtsp_reader(size_t frame_size)
{
    rtsp_frame_size = frame_size;
    rtsp_latest_frame.assign(frame_size, 0);
    rtsp_frame_ready = false;
    rtsp_frame_seq = 0;
    rtsp_last_consumed_seq = 0;
    rtsp_reader_running = true;

    if (pthread_create(&rtsp_reader_tid, nullptr, rtsp_reader_thread, nullptr) != 0) {
        rtsp_reader_running = false;
        return false;
    }
    rtsp_reader_started = true;
    return true;
}

bool reconnect_rtsp(size_t frame_size)
{
    printf("Reconnect RTSP stream...\n");
    stop_rtsp_reader();
    sleep(1);
    for (int retry = 0; retry < 10; retry++) {
        printf("Reconnect attempt %d...\n", retry + 1);
        if (open_rtsp_pipe() && start_rtsp_reader(frame_size)) {
            printf("RTSP reconnect success\n");
            return true;
        }
        if (ffmpeg_pipe) { pclose(ffmpeg_pipe); ffmpeg_pipe = nullptr; }
        ffmpeg_fd = -1;
        sleep(1);
    }
    printf("RTSP reconnect failed\n");
    return false;
}

// Wait for a NEW frame from reader thread and copy it.
//
// Timeout must be generous: an RTSP source can take several seconds to connect
// over TCP and emit its first keyframe. A short timeout here would tear down a
// still-connecting ffmpeg and loop forever. We only signal an error (→ reconnect)
// when the reader thread actually dies (ffmpeg exited / EOF) or after a long
// safety timeout.
bool grab_latest_frame(unsigned char *buf, size_t frame_size)
{
    if (!rtsp_reader_started || frame_size != rtsp_frame_size)
        return false;

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += 15; // safety backstop; normal trigger is reader thread death

    pthread_mutex_lock(&rtsp_mutex);
    while (rtsp_reader_running &&
           (!rtsp_frame_ready || rtsp_frame_seq == rtsp_last_consumed_seq)) {
        int rc = pthread_cond_timedwait(&rtsp_cond, &rtsp_mutex, &ts);
        if (rc == ETIMEDOUT) {
            pthread_mutex_unlock(&rtsp_mutex);
            return false;
        }
    }
    if (!rtsp_frame_ready || rtsp_frame_seq == rtsp_last_consumed_seq) {
        // Reader thread died (ffmpeg exited) — signal reconnect.
        pthread_mutex_unlock(&rtsp_mutex);
        return false;
    }

    memcpy(buf, rtsp_latest_frame.data(), frame_size);
    rtsp_last_consumed_seq = rtsp_frame_seq;
    pthread_mutex_unlock(&rtsp_mutex);

    return true;
}
#endif // USE_RTSP_INPUT

int main(int argc, char *argv[])
{
    system("RkLunch-stop.sh");

    RK_S32 s32Ret = 0;
    int sX, sY, eX, eY;
    char text[16];

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

#if USE_RTSP_INPUT
    if (!open_rtsp_pipe()) return -1;
    if (!start_rtsp_reader(frame_bytes)) {
        printf("Failed to start RTSP reader thread\n");
        pclose(ffmpeg_pipe);
        ffmpeg_pipe = nullptr;
        ffmpeg_fd = -1;
        return -1;
    }
#endif

    float fps      = 0.0f;
    char  fps_text[32];

    while (1)
    {
        // ------------------------------------------------------------------
        // Step 1: receive the LATEST available raw frame (skip stale ones)
        // ------------------------------------------------------------------
        h264_frame.stVFrame.u32TimeRef = H264_TimeRef++;
        h264_frame.stVFrame.u64PTS     = TEST_COMM_GetNowUs();

#if USE_RTSP_INPUT
        if (!grab_latest_frame(src_data, frame_bytes))
        {
            printf("RTSP frame read error\n");
            if (!reconnect_rtsp(frame_bytes)) { sleep(1); }
            continue;
        }

#else
        s32Ret = RK_MPI_VI_GetChnFrame(0, 0, &stViFrame, -1);
        if (s32Ret == RK_SUCCESS)
        {
            void *vi_data = RK_MPI_MB_Handle2VirAddr(stViFrame.stVFrame.pMbBlk);
            cv::Mat yuv420sp(height + height / 2, width, CV_8UC1, vi_data);
            cv::cvtColor(yuv420sp, src_frame, cv::COLOR_YUV420sp2RGB);
        }
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

        snprintf(fps_text, sizeof(fps_text), "FPS: %.2f", fps);
        cv::putText(draw_frame[cur], fps_text, cv::Point(20, 40),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0,
                    cv::Scalar(0, 0, 255), 2);

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
    stop_rtsp_reader();
#endif

    RK_MPI_SYS_Exit();
    release_yolov8_model(&rknn_app_ctx);
    deinit_post_process();
    return 0;
}