#include "mqtt_client.h"

#include <cstdio>
#include <cstring>
#include <vector>

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>

#define MQTT_KEEPALIVE      60
#define MQTT_QUEUE_MAX      128
#define MQTT_RECONNECT_SEC  3
#define MQTT_CONNECT_TMO_MS 1500

MqttClient::~MqttClient()
{
    stop();
}

void MqttClient::start()
{
    if (running_.exchange(true))
        return;
    thread_ = std::thread(&MqttClient::worker, this);
}

void MqttClient::stop()
{
    if (!running_.exchange(false))
        return;
    q_cv_.notify_all();
    if (thread_.joinable())
        thread_.join();
    do_disconnect();
}

void MqttClient::configure(const std::string &host, int port,
                           const std::string &user, const std::string &pass)
{
    std::lock_guard<std::mutex> lk(cfg_mtx_);
    if (host_ == host && port_ == port && user_ == user && pass_ == pass)
        return;
    host_ = host;
    port_ = port;
    user_ = user;
    pass_ = pass;
    cfg_gen_++;          // worker заметит изменение и переподключится
    q_cv_.notify_all();
}

void MqttClient::publish(const std::string &topic, const std::string &payload)
{
    {
        std::lock_guard<std::mutex> lk(q_mtx_);
        if (queue_.size() >= MQTT_QUEUE_MAX)
            queue_.pop_front();          // дропаем самое старое
        queue_.push_back(Msg{topic, payload});
    }
    q_cv_.notify_all();
}

bool MqttClient::send_all(const unsigned char *buf, size_t len)
{
    size_t done = 0;
    while (done < len) {
        ssize_t n = send(sock_, buf + done, len - done, MSG_NOSIGNAL);
        if (n > 0) { done += (size_t)n; continue; }
        if (n < 0 && errno == EINTR) continue;
        return false;
    }
    return true;
}

// Кодирование "remaining length" в формате MQTT (varint, 1-4 байта).
static size_t encode_remaining_length(unsigned char *out, size_t len)
{
    size_t i = 0;
    do {
        unsigned char b = len % 128;
        len /= 128;
        if (len > 0) b |= 0x80;
        out[i++] = b;
    } while (len > 0 && i < 4);
    return i;
}

void MqttClient::do_disconnect()
{
    if (sock_ >= 0) {
        ::close(sock_);
        sock_ = -1;
    }
    connected_.store(false);
}

bool MqttClient::do_connect()
{
    std::string host, user, pass;
    int port;
    unsigned gen;
    {
        std::lock_guard<std::mutex> lk(cfg_mtx_);
        host = host_; port = port_; user = user_; pass = pass_; gen = cfg_gen_;
    }
    conn_gen_ = gen;

    if (host.empty())
        return false;

    // --- DNS / resolve ---
    char portbuf[16];
    snprintf(portbuf, sizeof(portbuf), "%d", port);

    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family   = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    struct addrinfo *res = nullptr;
    if (getaddrinfo(host.c_str(), portbuf, &hints, &res) != 0 || !res) {
        printf("[mqtt] resolve failed: %s\n", host.c_str());
        return false;
    }

    int s = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (s < 0) { freeaddrinfo(res); return false; }

    // Неблокирующий connect с таймаутом, чтобы не зависнуть на мёртвом брокере.
    int flags = fcntl(s, F_GETFL, 0);
    fcntl(s, F_SETFL, flags | O_NONBLOCK);

    int rc = connect(s, res->ai_addr, res->ai_addrlen);
    if (rc < 0 && errno == EINPROGRESS) {
        fd_set wf;
        FD_ZERO(&wf);
        FD_SET(s, &wf);
        struct timeval tv;
        tv.tv_sec  = MQTT_CONNECT_TMO_MS / 1000;
        tv.tv_usec = (MQTT_CONNECT_TMO_MS % 1000) * 1000;
        rc = select(s + 1, nullptr, &wf, nullptr, &tv);
        if (rc <= 0) { ::close(s); freeaddrinfo(res); return false; }
        int err = 0; socklen_t el = sizeof(err);
        getsockopt(s, SOL_SOCKET, SO_ERROR, &err, &el);
        if (err != 0) { ::close(s); freeaddrinfo(res); return false; }
    } else if (rc < 0) {
        ::close(s); freeaddrinfo(res); return false;
    }
    freeaddrinfo(res);

    fcntl(s, F_SETFL, flags);            // обратно в блокирующий режим
    int one = 1;
    setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    struct timeval io_tv;
    io_tv.tv_sec = 2; io_tv.tv_usec = 0;
    setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &io_tv, sizeof(io_tv));
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &io_tv, sizeof(io_tv));

    sock_ = s;

    // --- CONNECT packet ---
    const std::string client_id = "luckfox-yolov8";
    bool has_user = !user.empty();
    bool has_pass = has_user && !pass.empty();

    std::vector<unsigned char> vh;
    auto put_str = [&](std::vector<unsigned char> &v, const std::string &str) {
        v.push_back((str.size() >> 8) & 0xFF);
        v.push_back(str.size() & 0xFF);
        v.insert(v.end(), str.begin(), str.end());
    };

    // variable header
    put_str(vh, "MQTT");
    vh.push_back(0x04);                   // protocol level 3.1.1
    unsigned char flags_byte = 0x02;      // clean session
    if (has_user) flags_byte |= 0x80;
    if (has_pass) flags_byte |= 0x40;
    vh.push_back(flags_byte);
    vh.push_back((MQTT_KEEPALIVE >> 8) & 0xFF);
    vh.push_back(MQTT_KEEPALIVE & 0xFF);
    // payload
    put_str(vh, client_id);
    if (has_user) put_str(vh, user);
    if (has_pass) put_str(vh, pass);

    unsigned char fixed[5];
    fixed[0] = 0x10;                      // CONNECT
    size_t rl = encode_remaining_length(fixed + 1, vh.size());

    if (!send_all(fixed, 1 + rl) || !send_all(vh.data(), vh.size())) {
        do_disconnect();
        return false;
    }

    // --- читаем CONNACK (4 байта) ---
    unsigned char ack[4];
    size_t got = 0;
    while (got < 4) {
        ssize_t n = recv(sock_, ack + got, 4 - got, 0);
        if (n > 0) { got += (size_t)n; continue; }
        if (n < 0 && errno == EINTR) continue;
        do_disconnect();
        return false;
    }
    if (ack[0] != 0x20 || ack[3] != 0x00) {
        printf("[mqtt] CONNACK rejected (rc=%d)\n", ack[3]);
        do_disconnect();
        return false;
    }

    connected_.store(true);
    printf("[mqtt] connected to %s:%d\n", host.c_str(), port);
    return true;
}

bool MqttClient::ensure_connected()
{
    // конфиг изменился => переподключаемся
    unsigned gen;
    bool empty_host;
    {
        std::lock_guard<std::mutex> lk(cfg_mtx_);
        gen = cfg_gen_;
        empty_host = host_.empty();
    }
    if (gen != conn_gen_)
        do_disconnect();

    if (empty_host) {            // брокер не задан — простаиваем
        do_disconnect();
        return false;
    }

    if (connected_.load())
        return true;

    time_t now = time(nullptr);
    if (now - last_attempt_ < MQTT_RECONNECT_SEC)
        return false;
    last_attempt_ = now;

    return do_connect();
}

bool MqttClient::send_publish(const Msg &m)
{
    std::vector<unsigned char> pkt;
    // variable header: topic
    std::vector<unsigned char> body;
    body.push_back((m.topic.size() >> 8) & 0xFF);
    body.push_back(m.topic.size() & 0xFF);
    body.insert(body.end(), m.topic.begin(), m.topic.end());
    // QoS0 => без packet id
    body.insert(body.end(), m.payload.begin(), m.payload.end());

    unsigned char fixed[5];
    fixed[0] = 0x30;                      // PUBLISH, QoS0
    size_t rl = encode_remaining_length(fixed + 1, body.size());

    if (!send_all(fixed, 1 + rl) || !send_all(body.data(), body.size())) {
        do_disconnect();
        return false;
    }
    return true;
}

void MqttClient::worker()
{
    while (running_.load()) {
        if (!ensure_connected()) {
            // нет соединения — ждём немного и пробуем снова
            std::unique_lock<std::mutex> lk(q_mtx_);
            q_cv_.wait_for(lk, std::chrono::milliseconds(500));
            continue;
        }

        Msg m;
        bool have = false;
        {
            std::unique_lock<std::mutex> lk(q_mtx_);
            if (queue_.empty())
                q_cv_.wait_for(lk, std::chrono::milliseconds(500));
            if (!queue_.empty()) {
                m = std::move(queue_.front());
                queue_.pop_front();
                have = true;
            }
        }
        if (have)
            send_publish(m);     // при ошибке do_disconnect() внутри
    }
}
