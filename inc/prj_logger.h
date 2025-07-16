#ifndef PROJECT_LOGGER_HPP
#define PROJECT_LOGGER_HPP

#include "../cxxlog/cxxlog.h"
#include "../cxxlog/cxx_attrubutes.h"  // ����� ��� ����� ���������

// ��������� ��� ������� (����� � �����, ���� ������)
typedef cxxlog::Log<Mode, Counter> ProjectLogger;

// ��������� ���������� ��������� ������� ��� 'extern'
extern ProjectLogger g_logger;

// ��������� ������� ������������� (���� ��� ����������)
extern void initialize_project_logger();

namespace app_log { // �������� ���� ������������ ���� ��� ���� �������

// ������� ��� ��������� ����������� ������� (���������� ���� ���)
// ���� ��� ����� ����������, ���� g_logger ������ ��������
// ProjectLoggerType* s_global_logger_ptr = nullptr;
// inline void set_global_logger(ProjectLoggerType& logger_instance) {
//     s_global_logger_ptr = &logger_instance;
// }

// �������-�������
template <Severity lvl, class... Args>
inline constexpr auto log(const Args&... args) {
    // if (!s_global_logger_ptr) return; // ��������, ���� ���������� s_global_logger_ptr
    // return print_log<lvl>(*s_global_logger_ptr, args...);
    // ���� g_logger - ��� ���������� ����������, ��������� �����:
    return print_log<lvl>(g_logger, args...);
}

// �����������, �������� ������
template <class... Args> inline constexpr auto trace(const Args&... args) { return log<LOG_TRACE>(args...); }
template <class... Args> inline constexpr auto debug(const Args&... args) { return log<LOG_DEBUG>(args...); }
template <class... Args> inline constexpr auto info(const Args&... args) { return log<LOG_INFO>(args...); }
template <class... Args> inline constexpr auto warning(const Args&... args) { return log<LOG_WARNING>(args...); }
template <class... Args> inline constexpr auto error(const Args&... args) { return log<LOG_ERROR>(args...); }
template <class... Args> inline constexpr auto fatal(const Args&... args) { return log<LOG_FATAL>(args...); }

} // namespace app_log

#endif // PROJECT_LOGGER_HPP
