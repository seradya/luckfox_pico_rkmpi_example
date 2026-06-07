#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <string>
#include <vector>
#include <mutex>

#include "mongoose.h"

// RTSP камеры (до 4). Изменяется веб-интерфейсом, читается пайплайном —
// любой доступ должен брать cameras_mutex.
extern std::vector<std::string> cameras;
extern std::mutex cameras_mutex;

// Генерация HTML страницы
std::string generate_html();

// HTTP обработчик mongoose
void handle_request(struct mg_connection *c, int ev, void *ev_data);

// Запускает HTTP-сервер (блокирующий цикл). Вызывать в отдельном потоке.
void run_web_server(int port);

// --- Хелперы для пайплайна (потокобезопасные) ---

// Перебор по кругу. На входе idx — индекс текущей камеры (или -1).
// Находит следующий непустой URL, обновляет idx и url, возвращает true.
// false — если ни одна камера не настроена.
bool get_next_camera(int &idx, std::string &url);

// Количество настроенных (непустых) камер.
int active_camera_count();

#endif // WEB_SERVER_H
