#ifndef PROJECT_LOGGER_HPP
#define PROJECT_LOGGER_HPP

#include "../cxxlog/cxxlog.h"
#include "../cxxlog/cxx_attrubutes.h"  // Нужен для типов атрибутов

// Объявляем тип логгера (можно и здесь, если удобно)
typedef cxxlog::Log<Mode, Counter> ProjectLogger;

// Объявляем глобальный экземпляр логгера как 'extern'
extern ProjectLogger g_logger;

// Объявляем функцию инициализации (если она глобальная)
extern void initialize_project_logger();

namespace app_log { // Создадим свое пространство имен для этих функций

// Функция для установки глобального логгера (вызывается один раз)
// Этот шаг можно пропустить, если g_logger всегда доступен
// ProjectLoggerType* s_global_logger_ptr = nullptr;
// inline void set_global_logger(ProjectLoggerType& logger_instance) {
//     s_global_logger_ptr = &logger_instance;
// }

// Функции-обертки
template <Severity lvl, class... Args>
inline constexpr auto log(const Args&... args) {
    // if (!s_global_logger_ptr) return; // Проверка, если используем s_global_logger_ptr
    // return print_log<lvl>(*s_global_logger_ptr, args...);
    // Если g_logger - это глобальная переменная, доступная здесь:
    return print_log<lvl>(g_logger, args...);
}

// Опционально, короткие версии
template <class... Args> inline constexpr auto trace(const Args&... args) { return log<LOG_TRACE>(args...); }
template <class... Args> inline constexpr auto debug(const Args&... args) { return log<LOG_DEBUG>(args...); }
template <class... Args> inline constexpr auto info(const Args&... args) { return log<LOG_INFO>(args...); }
template <class... Args> inline constexpr auto warning(const Args&... args) { return log<LOG_WARNING>(args...); }
template <class... Args> inline constexpr auto error(const Args&... args) { return log<LOG_ERROR>(args...); }
template <class... Args> inline constexpr auto fatal(const Args&... args) { return log<LOG_FATAL>(args...); }

} // namespace app_log

#endif // PROJECT_LOGGER_HPP
