#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include <string>
#include <deque>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

// Минимальный MQTT 3.1.1 publisher (QoS 0) без внешних зависимостей.
//
// Публикация полностью асинхронная: publish() лишь кладёт сообщение в очередь,
// а отдельный фоновый поток держит TCP-соединение с брокером, переподключается
// при сбоях и отправляет PUBLISH. Это важно, чтобы сетевые задержки/недоступный
// брокер НИКОГДА не блокировали видеопайплайн.
class MqttClient {
public:
    MqttClient() = default;
    ~MqttClient();

    void start();   // запустить фоновый поток
    void stop();    // остановить и закрыть соединение

    // Задать целевой брокер. Можно вызывать часто (например, каждый кадр):
    // переподключение происходит только если параметры реально изменились.
    // Пустой host => клиент отключается и простаивает.
    void configure(const std::string &host, int port,
                   const std::string &user, const std::string &pass);

    // Поставить сообщение в очередь (не блокирует). При переполнении очереди
    // самые старые сообщения отбрасываются.
    void publish(const std::string &topic, const std::string &payload);

    bool is_connected() const { return connected_.load(); }

private:
    struct Msg { std::string topic; std::string payload; };

    void worker();
    bool ensure_connected();
    bool do_connect();
    void do_disconnect();
    bool send_all(const unsigned char *buf, size_t len);
    bool send_publish(const Msg &m);

    // целевая конфигурация (защищена cfg_mtx_)
    std::mutex  cfg_mtx_;
    std::string host_;
    int         port_ = 1883;
    std::string user_;
    std::string pass_;
    unsigned    cfg_gen_ = 0;        // увеличивается при каждом изменении конфига

    // состояние соединения (только в потоке worker)
    int         sock_         = -1;
    unsigned    conn_gen_     = 0;   // поколение конфига текущего соединения
    time_t      last_attempt_ = 0;

    // очередь сообщений
    std::mutex                  q_mtx_;
    std::condition_variable     q_cv_;
    std::deque<Msg>             queue_;

    std::thread        thread_;
    std::atomic<bool>  running_{false};
    std::atomic<bool>  connected_{false};
};

#endif // MQTT_CLIENT_H
