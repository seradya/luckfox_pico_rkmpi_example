#include "web_server.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>

#include <unistd.h>
#include <sys/wait.h>
#include <signal.h>
#include <time.h>

#define SNAPSHOT_TIMEOUT_SEC 8

// Камеры (до 4). Доступ только под cameras_mutex.
std::vector<std::string> cameras = {
    "",
    "",
    "",
    ""
};
std::mutex cameras_mutex;

// Камера, чей инференс уходит в выходной стрим (по умолчанию 0).
std::atomic<int> g_stream_camera{0};

// Настройки экспорта (MQTT/JSON). Доступ только под mqtt_settings_mutex.
static MqttSettings g_mqtt_settings;
static std::mutex   g_mqtt_settings_mutex;

MqttSettings get_mqtt_settings() {
    std::lock_guard<std::mutex> lk(g_mqtt_settings_mutex);
    return g_mqtt_settings;
}

// Простейшие извлекатели значений из плоского JSON-тела запроса.
static std::string json_get_string(const std::string &body, const char *key,
                                   const std::string &def) {
    std::string pat = std::string("\"") + key + "\":\"";
    size_t p = body.find(pat);
    if (p == std::string::npos) return def;
    size_t s = p + pat.size();
    size_t e = body.find('"', s);
    if (e == std::string::npos) return def;
    return body.substr(s, e - s);
}

static int json_get_int(const std::string &body, const char *key, int def) {
    std::string pat = std::string("\"") + key + "\":";
    size_t p = body.find(pat);
    if (p == std::string::npos) return def;
    return atoi(body.c_str() + p + pat.size());
}

static bool json_get_bool(const std::string &body, const char *key, bool def) {
    std::string pat = std::string("\"") + key + "\":";
    size_t p = body.find(pat);
    if (p == std::string::npos) return def;
    return body.compare(p + pat.size(), 4, "true") == 0;
}

// Флаг успешной проверки (под тем же мьютексом).
static std::vector<bool> camera_ok = {
    false,
    false,
    false,
    false
};

static bool test_camera_connection(const std::string& url) {
    if (url.empty()) return false;

    std::string cmd =
        "ffmpeg -rtsp_transport tcp "
        "-rtsp_flags prefer_tcp "
        "-fflags +discardcorrupt+nobuffer "
        "-flags low_delay "
        "-analyzeduration 5000000 "
        "-probesize 5000000 "
        "-i \"" + url + "\" "
        "-frames:v 1 "
        "-f null "
        "-loglevel error "
        "null";

    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) return false;

    int status = pclose(pipe);

    return WIFEXITED(status) && WEXITSTATUS(status) == 0;
}

// Снимает один кадр и пишет его JPEG прямо в файл (его потом отдаёт
// mg_http_serve_file). Запись в файл + отдача файлом надёжнее ручной
// сборки HTTP-ответа: mongoose сам корректно проставит Content-Type/
// Content-Length и дошлёт тело полностью.
static bool capture_snapshot_to_file(const std::string& url, const std::string& path) {
    if (url.empty()) return false;

    // Снимок должен делаться быстро: он выполняется синхронно в обработчике
    // mongoose и на это время блокирует веб-сервер. Поэтому минимальные
    // analyzeduration/probesize и низкая задержка.
    std::string cmd =
        "ffmpeg -rtsp_transport tcp "
        "-loglevel error "
        "-fflags nobuffer -flags low_delay "
        "-analyzeduration 0 "
        "-probesize 200000 "
        "-i '" + url + "' "
        "-frames:v 1 "
        "-q:v 5 "
        "-y "
        "-f mjpeg '" + path + "'";

    printf("[SNAP CMD] %s\n", cmd.c_str());

    int rc = system(cmd.c_str());

    printf("[SNAP rc] %d\n", rc);

    return rc == 0;
}

std::string generate_html() {
    std::string html;

    html =
        "<!DOCTYPE html>"
        "<html>"
        "<head>"
        "<meta charset='UTF-8'>"
        "<title>RTSP Cameras</title>"
        "</head>"

        "<body style='font-family:Arial;background:#1a1a1a;color:#fff;margin:20px;'>"

        "<h1>RTSP Camera Control</h1>"

        "<div id='cams'></div>"

        "<div style='margin-top:20px;text-align:center;'>"
        "<h2>Preview</h2>"

        "<img id='snap' "
        "style='max-width:100%;border:2px solid #2196F3;'>"

        "</div>"

        "<div style='margin-top:20px;padding:10px;background:#2a2a2a;border-radius:5px;max-width:560px;'>"
        "<h2>Detection export (MQTT / Frigate JSON)</h2>"
        "<label><input type='checkbox' id='m_en'> Enable MQTT</label><br><br>"
        "Broker host: <input type='text' id='m_host' "
        "style='width:220px;background:#333;color:#fff;border:1px solid #555;padding:4px;'> "
        "Port: <input type='text' id='m_port' "
        "style='width:70px;background:#333;color:#fff;border:1px solid #555;padding:4px;'><br><br>"
        "User: <input type='text' id='m_user' "
        "style='width:150px;background:#333;color:#fff;border:1px solid #555;padding:4px;'> "
        "Pass: <input type='password' id='m_pass' "
        "style='width:150px;background:#333;color:#fff;border:1px solid #555;padding:4px;'><br><br>"
        "Base topic: <input type='text' id='m_topic' "
        "style='width:200px;background:#333;color:#fff;border:1px solid #555;padding:4px;'>"
        "<br><br>"
        "<label><input type='checkbox' id='m_json'> Save JSON file</label> "
        "dir: <input type='text' id='m_dir' "
        "style='width:200px;background:#333;color:#fff;border:1px solid #555;padding:4px;'>"
        "<br><br>"
        "<button onclick='saveMqtt()'>Save export settings</button>"
        "<span id='m_st' style='margin-left:10px'></span>"
        "</div>"

        "<script>";

    html += "var cams=[";

    {
        std::lock_guard<std::mutex> lk(cameras_mutex);
        for (size_t i = 0; i < cameras.size(); i++) {
            if (i > 0) {
                html += ",";
            }

            html += "\"";
            html += cameras[i];
            html += "\"";
        }
    }

    html += "];";

    html += "var stream=" + std::to_string(g_stream_camera.load()) + ";";

    {
        MqttSettings ms = get_mqtt_settings();
        html += "var mq={en:" + std::string(ms.mqtt_enabled ? "true" : "false") +
                ",host:\"" + ms.host + "\",port:" + std::to_string(ms.port) +
                ",user:\"" + ms.user + "\",pass:\"" + ms.pass +
                "\",topic:\"" + ms.base_topic +
                "\",json:" + std::string(ms.json_enabled ? "true" : "false") +
                ",dir:\"" + ms.json_dir + "\"};";
    }

    html +=
        "var tested=[];"
        "var saved=[];"

        "function init(){"
        "var d=document.getElementById('cams');"
        "var h='';"
        "for(var i=0;i<cams.length;i++){"
        "h+='<div style=\"margin:10px 0;padding:10px;background:#2a2a2a;border-radius:5px;\">';"
        "h+='Camera '+(i+1)+'<br>';"
        "h+='<input type=\"text\" id=\"url'+i+'\" value=\"'+cams[i]+'\" "
        "style=\"width:500px;background:#333;color:#fff;border:1px solid #555;padding:5px;\">';"
        "h+='<br><br>';"
        "h+='<button id=\"sv'+i+'\" onclick=\"saveCam('+i+')\">Save</button> ';"
        "h+='<button onclick=\"testCam('+i+')\">Test</button> ';"
        "h+='<button onclick=\"previewCam('+i+')\">Preview</button> ';"
        "h+='<button id=\"stm'+i+'\" onclick=\"streamCam('+i+')\">Stream</button> ';"
        "h+='<span id=\"st'+i+'\" style=\"margin-left:10px\"></span>';"
        "h+='</div>';"
        "}"
        "d.innerHTML=h;"
        "for(var i=0;i<cams.length;i++){if(cams[i])saved[i]=true;}"
        "refresh();"
        "}"

        // Подсветка: сохранённая камера -> зелёная кнопка Save;
        // активный выходной стрим -> синяя кнопка Stream.
        "function refresh(){"
        "for(var i=0;i<cams.length;i++){"
        "var sv=document.getElementById('sv'+i);"
        "if(sv){sv.style.background=saved[i]?'#2e7d32':'';sv.style.color=saved[i]?'#fff':'';}"
        "var stm=document.getElementById('stm'+i);"
        "if(stm){stm.style.background=(stream==i)?'#1565c0':'';stm.style.color=(stream==i)?'#fff':'';}"
        "}"
        "}"

        "function saveCam(i){"
        "cams[i]=document.getElementById('url'+i).value;"
        "tested[i]=false;"
        "var x=new XMLHttpRequest();"
        "x.open('POST','/save',true);"
        "x.setRequestHeader('Content-Type','application/json');"
        "x.onload=function(){saved[i]=(cams[i]!=='');refresh();};"
        "x.send(JSON.stringify({i:i,u:cams[i]}));"
        "document.getElementById('st'+i).innerHTML='Saved';"
        "}"

        "function testCam(i){"
        "document.getElementById('st'+i).innerHTML='Testing...';"
        "var x=new XMLHttpRequest();"
        "x.open('POST','/test',true);"
        "x.setRequestHeader('Content-Type','application/json');"
        "x.onload=function(){"
        "try{var r=JSON.parse(x.responseText);tested[i]=r.ok;"
        "document.getElementById('st'+i).innerHTML=r.ok?'OK':'FAIL';}"
        "catch(e){document.getElementById('st'+i).innerHTML='FAIL';}"
        "};"
        "x.send(JSON.stringify({u:document.getElementById('url'+i).value}));"
        "}"

        // Превью — один снимок по нажатию (без авто-обновления).
        "function previewCam(i){"
        "if(!tested[i]){alert('Test camera first');return;}"
        "document.getElementById('snap').src='/snap?id='+i+'&r='+Date.now();"
        "}"

        // Переключение выходного стрима на камеру i.
        "function streamCam(i){"
        "var x=new XMLHttpRequest();"
        "x.open('POST','/stream',true);"
        "x.setRequestHeader('Content-Type','application/json');"
        "x.onload=function(){stream=i;refresh();};"
        "x.send(JSON.stringify({i:i}));"
        "}"

        // Заполнить форму экспорта текущими настройками.
        "function mqttFill(){"
        "document.getElementById('m_en').checked=mq.en;"
        "document.getElementById('m_host').value=mq.host;"
        "document.getElementById('m_port').value=mq.port;"
        "document.getElementById('m_user').value=mq.user;"
        "document.getElementById('m_pass').value=mq.pass;"
        "document.getElementById('m_topic').value=mq.topic;"
        "document.getElementById('m_json').checked=mq.json;"
        "document.getElementById('m_dir').value=mq.dir;"
        "}"

        "function saveMqtt(){"
        "var o={"
        "enabled:document.getElementById('m_en').checked,"
        "host:document.getElementById('m_host').value,"
        "port:parseInt(document.getElementById('m_port').value)||1883,"
        "user:document.getElementById('m_user').value,"
        "pass:document.getElementById('m_pass').value,"
        "topic:document.getElementById('m_topic').value,"
        "jsonfile:document.getElementById('m_json').checked,"
        "dir:document.getElementById('m_dir').value"
        "};"
        "var x=new XMLHttpRequest();"
        "x.open('POST','/mqtt',true);"
        "x.setRequestHeader('Content-Type','application/json');"
        "x.onload=function(){document.getElementById('m_st').innerHTML='Saved';};"
        "x.send(JSON.stringify(o));"
        "}"

        "init();"
        "mqttFill();"

        "</script>"

        "</body>"
        "</html>";

    return html;
}

void handle_request(struct mg_connection *c,
                    int ev,
                    void *ev_data) {
    if (ev != MG_EV_HTTP_MSG) {
        return;
    }

    struct mg_http_message *hm =
        (struct mg_http_message *) ev_data;

    std::string uri(hm->uri.buf, hm->uri.len);
    uri = uri.substr(0, uri.find('?'));
    std::string method(hm->method.buf, hm->method.len);

    printf("%s %s\n", method.c_str(), uri.c_str());

    if (method == "GET" && uri == "/") {
        std::string html = generate_html();

        mg_http_reply(
            c,
            200,
            "Content-Type: text/html\r\n",
            "%s",
            html.c_str()
        );

        return;
    }

    if (method == "GET" && uri.rfind("/snap", 0) == 0) {
        char idbuf[32] = {0};

        mg_http_get_var(
            &hm->query,
            "id",
            idbuf,
            sizeof(idbuf)
        );

        int id = atoi(idbuf);

        // Копируем URL под мьютексом, а тяжёлый ffmpeg запускаем уже без него,
        // чтобы не блокировать пайплайн на время съёмки кадра.
        std::string url;
        {
            std::lock_guard<std::mutex> lk(cameras_mutex);
            if (id < 0 || id >= (int) cameras.size()) {
                mg_http_reply(c, 400, "", "bad id\n");
                return;
            }
            if (!camera_ok[id]) {
                mg_http_reply(c, 403, "", "camera not tested\n");
                return;
            }
            url = cameras[id];
        }

        std::string path = "/tmp/snap_" + std::to_string(id) + ".jpg";

        if (!capture_snapshot_to_file(url, path)) {
            mg_http_reply(c, 500, "", "snapshot failed\n");
            return;
        }

        struct mg_http_serve_opts opts;
        memset(&opts, 0, sizeof(opts));
        opts.mime_types    = "jpg=image/jpeg";
        opts.extra_headers = "Cache-Control: no-cache\r\n";

        mg_http_serve_file(c, hm, path.c_str(), &opts);

        return;
    }

    if (method == "POST" && uri == "/save") {
        std::string body(hm->body.buf, hm->body.len);

        size_t ipos = body.find("\"i\":");
        size_t upos = body.find("\"u\":\"");

        if (ipos != std::string::npos &&
            upos != std::string::npos) {

            int i = atoi(body.c_str() + ipos + 4);

            size_t us = upos + 5;
            size_t ue = body.find("\"", us);

            if (ue != std::string::npos &&
                i >= 0) {

                std::lock_guard<std::mutex> lk(cameras_mutex);
                if (i < (int) cameras.size()) {
                    cameras[i] = body.substr(us, ue - us);
                    camera_ok[i] = false;

                    mg_http_reply(
                        c,
                        200,
                        "Content-Type: application/json\r\n",
                        "{\"ok\":true}"
                    );

                    return;
                }
            }
        }

        mg_http_reply(
            c,
            200,
            "Content-Type: application/json\r\n",
            "{\"ok\":false}"
        );

        return;
    }

    if (method == "POST" && uri == "/test") {
        std::string body(hm->body.buf, hm->body.len);

        size_t upos = body.find("\"u\":\"");

        if (upos != std::string::npos) {
            size_t us = upos + 5;
            size_t ue = body.find("\"", us);

            if (ue != std::string::npos) {

                std::string url = body.substr(us, ue - us);

                bool ok = test_camera_connection(url);

                {
                    std::lock_guard<std::mutex> lk(cameras_mutex);
                    for (size_t i = 0; i < cameras.size(); i++) {
                        if (cameras[i] == url) {
                            camera_ok[i] = ok;
                        }
                    }
                }

                mg_http_reply(
                    c,
                    200,
                    "Content-Type: application/json\r\n",
                    "{\"ok\":%s}",
                    ok ? "true" : "false"
                );

                return;
            }
        }

        mg_http_reply(c, 200,
                      "Content-Type: application/json\r\n",
                      "{\"ok\":false}");
        return;
    }

    if (method == "POST" && uri == "/mqtt") {
        std::string body(hm->body.buf, hm->body.len);

        MqttSettings s;
        s.mqtt_enabled = json_get_bool(body, "enabled", false);
        s.host         = json_get_string(body, "host", "");
        s.port         = json_get_int(body, "port", 1883);
        s.user         = json_get_string(body, "user", "");
        s.pass         = json_get_string(body, "pass", "");
        s.base_topic   = json_get_string(body, "topic", "frigate");
        s.json_enabled = json_get_bool(body, "jsonfile", false);
        s.json_dir     = json_get_string(body, "dir", "/tmp");

        {
            std::lock_guard<std::mutex> lk(g_mqtt_settings_mutex);
            g_mqtt_settings = s;
        }

        printf("[mqtt] settings: mqtt=%d host=%s:%d topic=%s json=%d dir=%s\n",
               s.mqtt_enabled, s.host.c_str(), s.port, s.base_topic.c_str(),
               s.json_enabled, s.json_dir.c_str());

        mg_http_reply(c, 200, "Content-Type: application/json\r\n",
                      "{\"ok\":true}");
        return;
    }

    if (method == "POST" && uri == "/stream") {
        std::string body(hm->body.buf, hm->body.len);

        size_t ipos = body.find("\"i\":");
        if (ipos != std::string::npos) {
            int i = atoi(body.c_str() + ipos + 4);
            if (i >= 0 && i < (int) cameras.size()) {
                g_stream_camera.store(i);
                printf("[stream] output switched to camera %d\n", i);

                mg_http_reply(c, 200,
                              "Content-Type: application/json\r\n",
                              "{\"ok\":true}");
                return;
            }
        }

        mg_http_reply(c, 200,
                      "Content-Type: application/json\r\n",
                      "{\"ok\":false}");
        return;
    }

    mg_http_reply(c, 404, "", "Not found\n");
}

void run_web_server(int port) {
    struct mg_mgr mgr;
    mg_mgr_init(&mgr);

    std::string addr = "http://0.0.0.0:" + std::to_string(port);

    if (mg_http_listen(&mgr, addr.c_str(), handle_request, NULL) == NULL) {
        printf("web: failed to listen on %s\n", addr.c_str());
        mg_mgr_free(&mgr);
        return;
    }

    printf("Camera web server on http://<board-ip>:%d\n", port);

    for (;;) {
        mg_mgr_poll(&mgr, 100);
    }

    mg_mgr_free(&mgr);
}

bool get_next_camera(int &idx, std::string &url) {
    std::lock_guard<std::mutex> lk(cameras_mutex);

    int n = (int) cameras.size();
    if (n == 0) return false;

    int start = (idx < 0) ? -1 : idx;
    for (int step = 1; step <= n; step++) {
        int cand = ((start + step) % n + n) % n;
        if (!cameras[cand].empty()) {
            idx = cand;
            url = cameras[cand];
            return true;
        }
    }
    return false;
}

int active_camera_count() {
    std::lock_guard<std::mutex> lk(cameras_mutex);

    int cnt = 0;
    for (const auto &s : cameras) {
        if (!s.empty()) cnt++;
    }
    return cnt;
}

std::vector<std::string> get_all_cameras() {
    std::lock_guard<std::mutex> lk(cameras_mutex);
    return cameras;
}
